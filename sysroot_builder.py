"""クロスコンパイル用 sysroot を APT リポジトリから直接生成するモジュール。

multistrap や debootstrap に依存せず、apt-get の --download-only と
dpkg-deb --extract だけで sysroot を組み立てる。この構成には次の制約がある。

- root 権限を要求しない（chroot もパッケージの maintainer script も実行しない）
- ホストの APT 状態（/var/lib/apt など）を一切読み書きしない
- 設定ファイル (sysroot/*.json) と署名鍵だけから決定的に同じ sysroot を再現できる

maintainer script を実行しないため、通常はスクリプトが行う後処理
（usrmerge のシンボリックリンク作成など）は _postprocess_sysroot で自前で補う。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import cast
from urllib.parse import SplitResult, urlsplit

__all__ = [
    "RepositoryConfig",
    "SysrootBuildError",
    "SysrootConfig",
    "SysrootConfigError",
    "build_sysroot",
    "load_sysroot_config",
    "main",
    "sysroot_config_fingerprint",
]


# sysroot 直下に置く生成記録ファイル。設定のフィンガープリントを保存し、
# 次回のビルドで同一設定なら再生成をスキップする判定に使う。
# 定数名は cross repository で builder と生成物の形式を識別するための
# 恒久的な互換名として維持する。後処理や生成形式を変更したときは
# MANIFEST_VERSION と該当テストを同じ変更でインクリメントすること。
MANIFEST_NAME = ".webrtc-build-sysroot.json"

# sysroot の生成形式（後処理の内容など）を変えたらインクリメントする。
# 古い形式の sysroot は fingerprint が一致しても再利用しない。
MANIFEST_VERSION = 1

# 設定値のうち APT の設定ファイルやコマンドラインへ埋め込むものに許可する文字。
# sources.list の [] オプションや空白による区切りを壊す文字を弾き、
# 設定ファイル経由のインジェクションを構文レベルで防ぐ。
# `\` を許可するのは Windows 上のテスト実行で tmp_path が
# `C:\Users\...\keyring.gpg` を返しても validation に通すため。
# 実行時に sources.list へ埋まる signed_by は Linux 側の POSIX パスに限られる。
CONFIG_TOKEN_PATTERN = re.compile(r"^[A-Za-z0-9._+:/\\-]+$")

# arch は将来 arm64 以外へ拡張する余地を残しつつ、
# 現時点で検証済みでない表記を早期に弾く。amd64 / aarch64 などの別表記は本 builder のスコープ外。
_ALLOWED_ARCH = frozenset({"arm64"})

# repositories の各要素で受け付ける JSON キー。
# ここに無いキーは pin_priority などの綴り違いとして拒否する。
_REPOSITORY_ALLOWED_KEYS = frozenset({"url", "suite", "components", "signed_by", "pin_priority"})

# pin_priority の許容範囲。APT の pin は慣例的に 1..1000 の範囲で運用する。
_PIN_PRIORITY_MIN = 1
_PIN_PRIORITY_MAX = 1000


class SysrootConfigError(ValueError):
    """設定ファイル (sysroot/*.json) の内容が不正なときに送出するエラー。"""

    pass


class SysrootBuildError(RuntimeError):
    """設定は正しいが sysroot の生成処理が継続できないときに送出するエラー。"""

    pass


@dataclass(frozen=True)
class RepositoryConfig:
    """APT リポジトリ 1 つ分の設定。sources.list の 1 行に対応する。"""

    url: str
    suite: str
    components: tuple[str, ...]
    # Release ファイルの署名検証に使う鍵。設定読み込み時に絶対パスへ解決済み。
    signed_by: Path
    # url から抽出した hostname。pin 対象の特定に使うため設定読み込み時点で固定する。
    hostname: str
    # APT の pin priority。None のときはホスト側の既定挙動を変えない。
    # 数値のときは preferences で hostname 全体を対象に Pin-Priority を書き込む。
    pin_priority: int | None = None


@dataclass(frozen=True)
class SysrootConfig:
    """sysroot 1 つ分の設定。sysroot/*.json を検証済みの形で保持する。"""

    name: str
    # dpkg アーキテクチャ名 (arm64 など)
    arch: str
    # GNU トリプレット (aarch64-linux-gnu など)。ライブラリパスの解決に使う。
    triplet: str
    packages: tuple[str, ...]
    repositories: tuple[RepositoryConfig, ...]


# 以下の _require_* は JSON から読んだ値の検証ヘルパー。
# 不正な設定は APT 実行前の読み込み時点で SysrootConfigError として失敗させる。


def _require_object(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise SysrootConfigError(f"{label} must be a JSON object")
    if not all(isinstance(key, str) for key in value):
        raise SysrootConfigError(f"{label} must contain only string keys")
    return cast(dict[str, object], value)


def _require_string(value: object, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise SysrootConfigError(f"{label} must be a non-empty string")
    return value


def _require_token(value: object, label: str) -> str:
    token = _require_string(value, label)
    if CONFIG_TOKEN_PATTERN.fullmatch(token) is None:
        raise SysrootConfigError(f"{label} contains unsupported characters: {token}")
    return token


def _require_string_array(value: object, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value:
        raise SysrootConfigError(f"{label} must be a non-empty array")
    result = tuple(_require_string(item, f"{label}[]") for item in value)
    if len(set(result)) != len(result):
        raise SysrootConfigError(f"{label} must not contain duplicate values")
    return result


def _require_pin_priority(value: object, label: str) -> int:
    # bool は int のサブクラスなので isinstance(bool, int) が True になる。
    # 設定ファイルで True/False が数値扱いされるのは意図と外れるため明示的に拒否する。
    if isinstance(value, bool) or not isinstance(value, int):
        raise SysrootConfigError(f"{label} must be an integer")
    if value < _PIN_PRIORITY_MIN or value > _PIN_PRIORITY_MAX:
        raise SysrootConfigError(
            f"{label} must be between {_PIN_PRIORITY_MIN} and {_PIN_PRIORITY_MAX}: {value}"
        )
    return value


def _require_repository_hostname(url: str, parsed_url: SplitResult, label: str) -> str:
    # urlsplit の結果を素直に使いつつ、pin 対象の hostname として妥当かを確認する。
    # userinfo・query・fragment を含む URL は sources.list への埋め込みや pin の対象特定を
    # 曖昧にするため、pin の有無に関係なく設定読み込み時点で拒否する。
    if parsed_url.username is not None or parsed_url.password is not None:
        raise SysrootConfigError(f"{label}.url must not contain userinfo: {url}")
    if parsed_url.query:
        raise SysrootConfigError(f"{label}.url must not contain a query string: {url}")
    if parsed_url.fragment:
        raise SysrootConfigError(f"{label}.url must not contain a fragment: {url}")
    hostname = parsed_url.hostname
    if not hostname:
        raise SysrootConfigError(f"{label}.url must contain a hostname: {url}")
    return hostname


def _load_repository(value: object, config_dir: Path, index: int) -> RepositoryConfig:
    """repositories[index] を検証し、生成した RepositoryConfig を返す。"""
    label = f"repositories[{index}]"
    raw = _require_object(value, label)
    # 未知キーは pin_priority などの綴り違いを黙って無視しないためここで弾く。
    unknown_keys = set(raw.keys()) - _REPOSITORY_ALLOWED_KEYS
    if unknown_keys:
        raise SysrootConfigError(f"{label} contains unknown keys: {sorted(unknown_keys)}")
    url = _require_string(raw.get("url"), f"{label}.url")
    # 署名検証だけでなく通信経路も保護するため、HTTP への後退をここで拒否する。
    parsed_url = urlsplit(url)
    if parsed_url.scheme != "https" or not parsed_url.netloc:
        raise SysrootConfigError(f"{label}.url must be an HTTPS URL: {url}")
    # URL は sources.list の "deb [options] url suite components" 行へ埋め込むため、
    # 行の構文を壊す空白と、オプション区切りの角括弧を含む URL は受け付けない。
    if any(character.isspace() for character in url) or "[" in url or "]" in url:
        raise SysrootConfigError(f"{label}.url contains unsupported characters: {url}")
    # hostname 抽出と pin 対象特定の前提となる URL バリデーションを行う。
    hostname = _require_repository_hostname(url, parsed_url, label)

    suite = _require_token(raw.get("suite"), f"{label}.suite")
    components = tuple(
        _require_token(component, f"{label}.components[]")
        for component in _require_string_array(raw.get("components"), f"{label}.components")
    )
    # 署名鍵の相対パスは設定ファイルの配置場所を基準に解決し、
    # 実行時のカレントディレクトリに依存させない。
    signed_by_value = _require_string(raw.get("signed_by"), f"{label}.signed_by")
    signed_by = Path(signed_by_value)
    if not signed_by.is_absolute():
        signed_by = config_dir / signed_by
    signed_by = signed_by.resolve()
    if not signed_by.is_file():
        raise SysrootConfigError(f"{label}.signed_by does not exist: {signed_by}")
    # 解決後の絶対パスも sources.list の signed-by= オプションへ埋め込むため、
    # checkout の置き場所に構文を壊す文字が含まれていないかまで確認する。
    if CONFIG_TOKEN_PATTERN.fullmatch(str(signed_by)) is None:
        raise SysrootConfigError(f"{label}.signed_by contains unsupported characters: {signed_by}")

    pin_priority: int | None = None
    if "pin_priority" in raw:
        pin_priority = _require_pin_priority(raw["pin_priority"], f"{label}.pin_priority")

    return RepositoryConfig(
        url=url,
        suite=suite,
        components=components,
        signed_by=signed_by,
        hostname=hostname,
        pin_priority=pin_priority,
    )


def _validate_pin_priority_consistency(repositories: tuple[RepositoryConfig, ...]) -> None:
    # APT の pin は hostname 全体に作用するため、同じ hostname に対して
    # 異なる pin_priority を与えても一方しか有効にならない。設定側の混在を
    # 検証時点で弾き、意図しない優先付けを避ける。
    seen: dict[str, int | None] = {}
    for repository in repositories:
        if repository.hostname in seen:
            if seen[repository.hostname] != repository.pin_priority:
                raise SysrootConfigError(
                    "repositories with the same hostname must share pin_priority: "
                    f"{repository.hostname}"
                )
        else:
            seen[repository.hostname] = repository.pin_priority


def load_sysroot_config(path: Path) -> SysrootConfig:
    """sysroot 設定 JSON を読み込み、検証済みの SysrootConfig を返す。"""
    try:
        raw_value: object = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SysrootConfigError(f"Failed to read sysroot config: {path}: {error}") from error

    raw = _require_object(raw_value, "config")
    name = _require_token(raw.get("name"), "name")
    arch = _require_token(raw.get("arch"), "arch")
    if arch not in _ALLOWED_ARCH:
        raise SysrootConfigError(f"arch must be one of {sorted(_ALLOWED_ARCH)}: {arch}")
    triplet = _require_token(raw.get("triplet"), "triplet")
    packages = tuple(
        _require_token(package, "packages[]")
        for package in _require_string_array(raw.get("packages"), "packages")
    )

    repositories_value = raw.get("repositories")
    if not isinstance(repositories_value, list) or not repositories_value:
        raise SysrootConfigError("repositories must be a non-empty array")
    repositories = tuple(
        _load_repository(repository, path.parent, index)
        for index, repository in enumerate(repositories_value)
    )
    _validate_pin_priority_consistency(repositories)

    return SysrootConfig(
        name=name,
        arch=arch,
        triplet=triplet,
        packages=packages,
        repositories=repositories,
    )


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        while chunk := file.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def sysroot_config_fingerprint(config: SysrootConfig) -> str:
    """設定内容から sysroot の同一性を判定するためのハッシュを計算する。

    manifest に保存したこの値と比較して、設定が変わっていなければ
    既存 sysroot を再利用する。署名鍵は checkout の絶対パスではなく
    ファイル内容のハッシュで表現し、checkout の場所が違っても
    同一内容なら同じ fingerprint になるようにする。
    """
    repositories_payload: list[dict[str, object]] = []
    for repository in config.repositories:
        # pin_priority が指定された repository だけ payload に同 field を追加する。
        # 未指定 repository へ null を差し込むと、以前 pin を持たなかった設定の
        # fingerprint まで巻き添えで変わってしまうため、キーごと省く。
        repository_payload: dict[str, object] = {
            "url": repository.url,
            "suite": repository.suite,
            "components": repository.components,
            "signed_by_sha256": _file_sha256(repository.signed_by),
        }
        if repository.pin_priority is not None:
            repository_payload["pin_priority"] = repository.pin_priority
        repositories_payload.append(repository_payload)
    payload = {
        "name": config.name,
        "arch": config.arch,
        "triplet": config.triplet,
        "packages": config.packages,
        "repositories": repositories_payload,
    }
    # キー順序と区切り文字を固定した正規化 JSON をハッシュ対象にする。
    encoded = json.dumps(payload, ensure_ascii=True, separators=(",", ":"), sort_keys=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _require_command(name: str) -> str:
    path = shutil.which(name)
    if path is None:
        raise SysrootBuildError(f"Required command was not found: {name}")
    return path


def _run_command(
    args: list[str], *, environment: dict[str, str] | None = None, log_command: bool = True
) -> None:
    # log_command=False はパッケージごとの展開のようにログが冗長になる場合に使う。
    if log_command:
        logging.info("Running command: %s", shlex.join(args))
    subprocess.run(args, check=True, env=environment)


def _collect_pin_stanzas(config: SysrootConfig) -> list[str]:
    # 同一 hostname は _validate_pin_priority_consistency で一意に揃えているため、
    # 重複した stanza を出さないよう hostname 単位で集約する。
    seen: dict[str, int] = {}
    for repository in config.repositories:
        if repository.pin_priority is None:
            continue
        seen.setdefault(repository.hostname, repository.pin_priority)
    stanzas: list[str] = []
    for hostname, priority in seen.items():
        stanzas.append(
            "\n".join(
                [
                    "Package: *",
                    f'Pin: origin "{hostname}"',
                    f"Pin-Priority: {priority}",
                ]
            )
        )
    return stanzas


@dataclass(frozen=True)
class _AptLayout:
    """apt-get が読み書きする隔離ファイルとディレクトリの配置。

    _write_apt_files が生成した実体と _apt_options が渡す `-o` 値を
    1 か所で束ねるため、両者はこのオブジェクト経由でのみパスを参照する。
    """

    work_dir: Path
    state_dir: Path
    status_file: Path
    cache_dir: Path
    sources_list: Path
    apt_conf: Path
    preferences: Path
    preferences_parts: Path


def _make_apt_layout(work_dir: Path, preferences: Path) -> _AptLayout:
    """work_dir と preferences から共通の layout を組み立てる。"""
    state_dir = work_dir / "state"
    return _AptLayout(
        work_dir=work_dir,
        state_dir=state_dir,
        status_file=state_dir / "status",
        cache_dir=state_dir / "cache",
        sources_list=work_dir / "sources.list",
        apt_conf=work_dir / "apt.conf",
        preferences=preferences,
        preferences_parts=work_dir / "preferencesparts",
    )


def _apt_options(layout: _AptLayout) -> list[str]:
    # APT の状態ディレクトリ・キャッシュ・ソース定義をすべて work_dir 配下へ隔離し、
    # ホストの /var/lib/apt や /etc/apt を読み書きしないようにする。
    # これにより root 権限なしで、ホスト環境を汚さずに apt-get を実行できる。
    return [
        "-o",
        f"Dir::State={layout.state_dir}",
        "-o",
        f"Dir::State::status={layout.status_file}",
        "-o",
        f"Dir::Cache={layout.cache_dir}",
        "-o",
        f"Dir::Etc::sourcelist={layout.sources_list}",
        "-o",
        "Dir::Etc::sourceparts=/dev/null",
        "-o",
        f"Dir::Etc::preferences={layout.preferences}",
        "-o",
        f"Dir::Etc::preferencesparts={layout.preferences_parts}",
        "-o",
        # 隔離した状態ディレクトリではロック取得そのものを止める。
        # 他の apt-get と物理的に共有しない前提のため排他制御は不要。
        "Debug::NoLocking=true",
    ]


def _write_apt_files(config: SysrootConfig, work_dir: Path) -> _AptLayout:
    """apt-get が参照する隔離ファイルを work_dir 配下に生成し、layout を返す。"""
    # pin の有無で preferences のパスだけが変わる。先に決定してから
    # layout を 1 度だけ組み立て、以後の書き込みは全て同じ layout を通す。
    stanzas = _collect_pin_stanzas(config)
    if stanzas:
        preferences: Path = work_dir / "preferences"
    else:
        # /dev/null に向けてホスト側の /etc/apt/preferences を確実に無効化する。
        preferences = Path("/dev/null")
    layout = _make_apt_layout(work_dir, preferences)

    # apt-get が期待するディレクトリ構造を用意する。
    # 空の status ファイルは「何もインストールされていないシステム」を意味し、
    # install が依存パッケージを漏れなく解決・ダウンロードする前提になる。
    (layout.state_dir / "lists" / "partial").mkdir(parents=True)
    (layout.cache_dir / "archives" / "partial").mkdir(parents=True)
    layout.status_file.touch()
    # preferencesparts はホスト側の設定を読ませないための空ディレクトリとして先に用意する。
    layout.preferences_parts.mkdir()

    # ホストの /etc/apt を読ませず、対象アーキテクチャだけを見る apt.conf を生成する。
    # APT::Architecture を設定ファイル側で固定することで、
    # x86_64 ホスト上でも arm64 などのパッケージを解決できる。
    apt_config = "\n".join(
        [
            'Dir::Etc::main "/dev/null";',
            'Dir::Etc::parts "/dev/null";',
            f'APT::Architecture "{config.arch}";',
            f'APT::Architectures {{ "{config.arch}"; }};',
            'Acquire::Languages "none";',
            'APT::Install-Recommends "false";',
            'APT::Install-Suggests "false";',
            "",
        ]
    )
    layout.apt_conf.write_text(apt_config, encoding="utf-8")

    source_lines = []
    for repository in config.repositories:
        components = " ".join(repository.components)
        source_lines.append(
            f"deb [arch={config.arch} signed-by={repository.signed_by}] "
            f"{repository.url} {repository.suite} {components}"
        )
    layout.sources_list.write_text("\n".join(source_lines) + "\n", encoding="utf-8")

    if stanzas:
        layout.preferences.write_text("\n\n".join(stanzas) + "\n", encoding="utf-8")
    return layout


def _ensure_usrmerge_symlinks(root: Path) -> None:
    # dpkg-deb --extract は maintainer script を実行しないため、
    # 通常は usrmerge パッケージが作成する /lib -> usr/lib などのリンクが存在しない。
    # このリンクがないと、パッケージが /lib/... を参照するパスで配置したファイルと
    # /usr/lib/... を参照するリンカがすれ違って解決に失敗するため、ここで補う。
    for legacy, merged in (
        ("bin", "usr/bin"),
        ("sbin", "usr/sbin"),
        ("lib", "usr/lib"),
        ("lib64", "usr/lib64"),
    ):
        legacy_path = root / legacy
        # パッケージが実体のディレクトリやリンクを配置済みの場合はそれを尊重する。
        if legacy_path.is_symlink() or legacy_path.exists():
            continue
        if (root / merged).is_dir():
            legacy_path.symlink_to(merged)


def _fix_absolute_symlinks(root: Path) -> None:
    # パッケージ内の絶対パスリンク (例: libfoo.so -> /usr/lib/.../libfoo.so.1) は
    # sysroot の外、つまりホスト側のファイルを指してしまう。
    # クロスコンパイル時にリンカが正しいライブラリを解決できるよう、
    # sysroot 内で完結する相対リンクへ張り替える。
    root_resolved = root.resolve()
    for path in root.rglob("*"):
        if not path.is_symlink():
            continue
        target = Path(os.readlink(path))
        if not target.is_absolute():
            continue
        # /etc/alternatives 経由のリンクなど、展開だけでは実体が存在しない
        # リンク先は張り替えの根拠がないためそのまま残す。
        target_in_sysroot = root / target.relative_to("/")
        if not target_in_sysroot.exists():
            continue
        # 絶対 symlink のターゲットが `..` を含み、sysroot の外へ逃げるケースを弾く。
        # 署名付きリポジトリを信頼する前提でも、後処理が sysroot 境界を越えないことは
        # defense-in-depth として担保する。
        resolved_target = target_in_sysroot.resolve()
        if not resolved_target.is_relative_to(root_resolved):
            continue
        # 相対 path も resolve 済みの値から作り、書き込む symlink に `..` が残らないようにする。
        relative_target = os.path.relpath(resolved_target, start=path.parent.resolve())
        path.unlink()
        path.symlink_to(relative_target)


def _link_pkgconfig_files(root: Path, triplet: str) -> None:
    # WebRTC のビルドは usr/share/pkgconfig から .pc ファイルを探すため、
    # トリプレット固有ディレクトリにしかない定義へ互換リンクを張る。
    source_dir = root / "usr" / "lib" / triplet / "pkgconfig"
    if not source_dir.is_dir():
        return
    destination_dir = root / "usr" / "share" / "pkgconfig"
    destination_dir.mkdir(parents=True, exist_ok=True)
    for source in sorted(source_dir.iterdir()):
        # pkg-config が実際に読むのは .pc のみ。関係無いファイルまで symlink しない。
        if source.suffix != ".pc":
            continue
        destination = destination_dir / source.name
        # パッケージが usr/share/pkgconfig へ直接配置した定義を上書きしない。
        if destination.exists() or destination.is_symlink():
            continue
        # sysroot を移動しても壊れないよう相対パスでリンクする。
        destination.symlink_to(f"../../lib/{triplet}/pkgconfig/{source.name}")


def _postprocess_sysroot(root: Path, triplet: str) -> None:
    # maintainer script を実行しない展開方式の穴を埋める後処理をまとめて行う。
    # 後処理の内容を変えたら MANIFEST_VERSION をインクリメントすること。
    _ensure_usrmerge_symlinks(root)
    _fix_absolute_symlinks(root)
    _link_pkgconfig_files(root, triplet)


def _read_manifest(output_dir: Path) -> dict[str, object] | None:
    # manifest が読めない・形式が不正な場合は None を返し、
    # 呼び出し側で「由来不明の既存ディレクトリ」として扱う。
    manifest_path = output_dir / MANIFEST_NAME
    try:
        raw_value: object = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(raw_value, dict) or not all(isinstance(key, str) for key in raw_value):
        return None
    return cast(dict[str, object], raw_value)


def _install_completed_sysroot(new_root: Path, output_dir: Path) -> None:
    # 完成した sysroot を出力先へ atomic に切り替える。
    # backup_dir は output_dir と同じ親に置く。作業用 TemporaryDirectory 内に置くと
    # 復元経路が二重に失敗したときに cleanup で既存が巻き添え削除される事故を招く。
    # BaseException を捕捉するのは Ctrl+C を含めても rollback を試みるため。
    backup_dir = output_dir.parent / f".{output_dir.name}.previous"
    # 前回の実行が rollback 失敗などで残した backup が残っていると、
    # 今回の output_dir.rename(backup_dir) が silently 失敗する。
    # 由来不明の残骸として明示的に拒否し、人手で除去する運用へ倒す。
    if backup_dir.exists() or backup_dir.is_symlink():
        raise SysrootBuildError(
            f"Stale backup directory remains from a previous failed run: {backup_dir}; "
            "remove it manually before retrying"
        )
    had_previous = output_dir.exists() or output_dir.is_symlink()
    if had_previous:
        output_dir.rename(backup_dir)
    try:
        new_root.rename(output_dir)
    except BaseException:
        # 新規の配置に失敗した場合は退避した既存の出力を元へ戻す。
        if had_previous:
            backup_dir.rename(output_dir)
        raise
    if had_previous:
        if backup_dir.is_dir() and not backup_dir.is_symlink():
            shutil.rmtree(backup_dir)
        else:
            # 既存の出力が symlink や通常ファイルだった場合は unlink で消す。
            backup_dir.unlink()


def build_sysroot(config: SysrootConfig, output_dir: Path, *, force: bool = False) -> bool:
    """設定に従って sysroot を output_dir へ生成する。

    一致する manifest を持つ sysroot が既にあれば再生成せず False を返す。
    実際に生成した場合は True を返す。設定と一致しない既存の出力は、
    force が指定されない限り黙って削除も再利用もせずエラーにする。
    """
    fingerprint = sysroot_config_fingerprint(config)
    manifest = _read_manifest(output_dir)
    if (
        not force
        and manifest is not None
        and manifest.get("format_version") == MANIFEST_VERSION
        and manifest.get("fingerprint") == fingerprint
    ):
        logging.info("Reusing sysroot: %s", output_dir)
        return False
    if not force and (output_dir.exists() or output_dir.is_symlink()):
        raise SysrootBuildError(
            f"Existing sysroot does not match the current config: {output_dir}; use --force"
        )

    apt_get = _require_command("apt-get")
    dpkg_deb = _require_command("dpkg-deb")
    output_dir.parent.mkdir(parents=True, exist_ok=True)

    # 作業ディレクトリは output_dir と同じ親に作り、
    # 完成後の rename による入れ替えが同一ファイルシステム内で完結するようにする。
    with tempfile.TemporaryDirectory(
        prefix=f".{output_dir.name}-", dir=output_dir.parent
    ) as temporary_dir_value:
        temporary_dir = Path(temporary_dir_value)
        # TemporaryDirectory は 0700 で作られるため、
        # sysroot をそのまま参照しても支障がないよう権限を緩める。
        temporary_dir.chmod(0o755)
        work_dir = temporary_dir / "apt"
        new_root = temporary_dir / "rootfs"
        work_dir.mkdir()
        new_root.mkdir()
        layout = _write_apt_files(config, work_dir)

        environment = os.environ.copy()
        environment["APT_CONFIG"] = str(layout.apt_conf)
        apt_options = _apt_options(layout)
        _run_command([apt_get, *apt_options, "update"], environment=environment)
        # --download-only により、依存解決とダウンロードだけを apt-get に任せる。
        # インストール（＝maintainer script の実行）はしないので root 権限が要らない。
        _run_command(
            [
                apt_get,
                *apt_options,
                "--download-only",
                "--yes",
                "--no-install-recommends",
                "--no-install-suggests",
                "install",
                *config.packages,
            ],
            environment=environment,
        )

        # ダウンロード済みの deb をすべて sysroot へ展開する。
        # dpkg-deb --extract はファイルを取り出すだけで maintainer script を実行しない。
        archive_dir = layout.cache_dir / "archives"
        deb_files = sorted(archive_dir.glob("*.deb"))
        if not deb_files:
            raise SysrootBuildError(f"No deb packages were downloaded for: {config.name}")
        logging.info("Extracting %d packages", len(deb_files))
        for deb_file in deb_files:
            _run_command([dpkg_deb, "--extract", str(deb_file), str(new_root)], log_command=False)

        _postprocess_sysroot(new_root, config.triplet)
        # 生成が最後まで完了した sysroot にだけ manifest を書き込む。
        # 途中で失敗した出力には manifest がないため、誤って再利用されることはない。
        manifest_value = {
            "format_version": MANIFEST_VERSION,
            "fingerprint": fingerprint,
            "name": config.name,
            "arch": config.arch,
            "triplet": config.triplet,
            "deb_files": [deb_file.name for deb_file in deb_files],
        }
        (new_root / MANIFEST_NAME).write_text(
            json.dumps(manifest_value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        _install_completed_sysroot(new_root, output_dir)

    logging.info("Built sysroot: %s", output_dir)
    return True


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a cross-compilation sysroot from an APT repository",
    )
    parser.add_argument(
        "--config",
        required=True,
        type=Path,
        help="Path to a sysroot config JSON file",
    )
    parser.add_argument(
        "--dest",
        required=True,
        type=Path,
        help="Destination directory for the generated sysroot",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replace an existing directory whose provenance does not match",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI エントリポイント。パース → 検証 → 生成の順で処理する。"""
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    parser = _build_argument_parser()
    args = parser.parse_args(argv)
    # subprocess で走る apt-get / dpkg-deb がカレント依存で迷子にならないよう絶対化する。
    config_path: Path = args.config.resolve()
    dest_path: Path = args.dest.resolve()
    force: bool = args.force
    try:
        config = load_sysroot_config(config_path)
        # 設定 JSON の name と実ファイルの stem を突き合わせ、
        # 誤ったペア（他ターゲットの設定を渡してしまう等）を早期に検出する。
        expected_stem = config_path.stem
        if config.name != expected_stem:
            raise SysrootConfigError(
                f"config name does not match file stem: {config.name} != {expected_stem}"
            )
        build_sysroot(config, dest_path, force=force)
    except (SysrootConfigError, SysrootBuildError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    except subprocess.CalledProcessError as error:
        # log_command=False で走らせた dpkg-deb などのため、失敗コマンドの basename を残す。
        program = os.path.basename(error.cmd[0]) if error.cmd else "command"
        print(
            f"error: {program} failed with exit code {error.returncode}",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
