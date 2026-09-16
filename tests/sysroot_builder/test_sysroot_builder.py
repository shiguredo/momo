from __future__ import annotations

import json
import os
from pathlib import Path
from typing import cast

import pytest

from sysroot_builder import (
    SysrootBuildError,
    SysrootConfigError,
    _collect_pin_stanzas,
    _fix_absolute_symlinks,
    _link_pkgconfig_files,
    build_sysroot,
    load_sysroot_config,
    sysroot_config_fingerprint,
)


def write_config(path: Path, *, name: str = "raspberry-pi-os_armv8") -> None:
    # 実運用の設定と同じ構造を使い、設定ファイルからの相対パス解決も確認できるようにする。
    config = {
        "name": name,
        "arch": "arm64",
        "triplet": "aarch64-linux-gnu",
        "packages": ["libc6-dev", "libstdc++-14-dev"],
        "repositories": [
            {
                "url": "https://deb.debian.org/debian",
                "suite": "trixie",
                "components": ["main"],
                "signed_by": "keyrings/debian-archive-keyring.gpg",
            }
        ],
    }
    path.write_text(json.dumps(config), encoding="utf-8")


def test_load_sysroot_config_resolves_relative_keyring(tmp_path: Path) -> None:
    # 署名鍵は設定ファイルの配置場所を基準に解決し、実行時の cwd に依存させない。
    config_path = tmp_path / "raspberry-pi-os_armv8.json"
    keyring_path = tmp_path / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir()
    keyring_path.touch()
    write_config(config_path)

    config = load_sysroot_config(config_path)

    assert config.name == "raspberry-pi-os_armv8"
    assert config.arch == "arm64"
    assert config.triplet == "aarch64-linux-gnu"
    assert config.packages == ("libc6-dev", "libstdc++-14-dev")
    assert config.repositories[0].signed_by == keyring_path


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("name", ""),
        ("arch", ""),
        ("triplet", ""),
        ("packages", []),
        ("repositories", []),
    ],
    ids=["empty-name", "empty-arch", "empty-triplet", "empty-packages", "empty-repositories"],
)
def test_load_sysroot_config_rejects_empty_required_values(
    tmp_path: Path, field: str, value: str | list[object]
) -> None:
    # 不完全な設定で APT を実行せず、設定の読み込み時点で明確に失敗させる。
    config_path = tmp_path / "invalid.json"
    write_config(config_path)
    keyring_path = tmp_path / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir()
    keyring_path.touch()
    raw_config: object = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw_config, dict):
        raise AssertionError("テスト設定は JSON オブジェクトである必要がある")
    raw_config = cast(dict[str, object], raw_config)
    raw_config[field] = value
    config_path.write_text(json.dumps(raw_config), encoding="utf-8")

    with pytest.raises(SysrootConfigError):
        load_sysroot_config(config_path)


def test_load_sysroot_config_rejects_insecure_repository_url(tmp_path: Path) -> None:
    # 署名検証だけでなく通信経路も保護し、HTTP への意図しない後退を設定読み込み時に拒否する。
    config_path = tmp_path / "invalid.json"
    keyring_path = tmp_path / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir()
    keyring_path.touch()
    write_config(config_path)
    raw_config: object = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw_config, dict):
        raise AssertionError("テスト設定は JSON オブジェクトである必要がある")
    raw_config = cast(dict[str, object], raw_config)
    repositories = raw_config.get("repositories")
    if (
        not isinstance(repositories, list)
        or not repositories
        or not isinstance(repositories[0], dict)
    ):
        raise AssertionError("テスト設定の repositories が不正")
    repository = cast(dict[str, object], repositories[0])
    repository["url"] = "http://deb.debian.org/debian"
    config_path.write_text(json.dumps(raw_config), encoding="utf-8")

    with pytest.raises(SysrootConfigError):
        load_sysroot_config(config_path)


@pytest.mark.parametrize(
    "pin_priority",
    [0, 1001, -1],
    ids=["zero", "over-max", "negative"],
)
def test_load_sysroot_config_rejects_pin_priority_out_of_range(
    tmp_path: Path, pin_priority: int
) -> None:
    # pin_priority の許容範囲を超える値を、APT 実行前の設定読み込み時点で弾く。
    config_path = tmp_path / "invalid.json"
    keyring_path = tmp_path / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir()
    keyring_path.touch()
    write_config(config_path)
    raw_config: object = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw_config, dict):
        raise AssertionError("テスト設定は JSON オブジェクトである必要がある")
    raw_config = cast(dict[str, object], raw_config)
    repositories = raw_config.get("repositories")
    if (
        not isinstance(repositories, list)
        or not repositories
        or not isinstance(repositories[0], dict)
    ):
        raise AssertionError("テスト設定の repositories が不正")
    repository = cast(dict[str, object], repositories[0])
    repository["pin_priority"] = pin_priority
    config_path.write_text(json.dumps(raw_config), encoding="utf-8")

    with pytest.raises(SysrootConfigError):
        load_sysroot_config(config_path)


def test_load_sysroot_config_rejects_pin_priority_boolean(tmp_path: Path) -> None:
    # bool は int のサブクラスだが、pin_priority に True/False を通すのは意図と外れる。
    config_path = tmp_path / "invalid.json"
    keyring_path = tmp_path / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir()
    keyring_path.touch()
    write_config(config_path)
    raw_config: object = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw_config, dict):
        raise AssertionError("テスト設定は JSON オブジェクトである必要がある")
    raw_config = cast(dict[str, object], raw_config)
    repositories = raw_config.get("repositories")
    if (
        not isinstance(repositories, list)
        or not repositories
        or not isinstance(repositories[0], dict)
    ):
        raise AssertionError("テスト設定の repositories が不正")
    repository = cast(dict[str, object], repositories[0])
    repository["pin_priority"] = True
    config_path.write_text(json.dumps(raw_config), encoding="utf-8")

    with pytest.raises(SysrootConfigError):
        load_sysroot_config(config_path)


def test_load_sysroot_config_rejects_unknown_repository_key(tmp_path: Path) -> None:
    # pin_priority の綴り違いなど、未知キーを黙って無視しないことを確認する。
    config_path = tmp_path / "invalid.json"
    keyring_path = tmp_path / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir()
    keyring_path.touch()
    write_config(config_path)
    raw_config: object = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw_config, dict):
        raise AssertionError("テスト設定は JSON オブジェクトである必要がある")
    raw_config = cast(dict[str, object], raw_config)
    repositories = raw_config.get("repositories")
    if (
        not isinstance(repositories, list)
        or not repositories
        or not isinstance(repositories[0], dict)
    ):
        raise AssertionError("テスト設定の repositories が不正")
    repository = cast(dict[str, object], repositories[0])
    # 綴り違いを想定した未知キーを追加する。
    repository["pin-priority"] = 990
    config_path.write_text(json.dumps(raw_config), encoding="utf-8")

    with pytest.raises(SysrootConfigError):
        load_sysroot_config(config_path)


def test_load_sysroot_config_rejects_conflicting_pin_priority_for_same_hostname(
    tmp_path: Path,
) -> None:
    # APT の pin は hostname 単位で作用するため、同一 hostname に異なる priority を許すと
    # 一方が黙って無効化される。設定読み込み時点で明確に失敗させる。
    config_path = tmp_path / "invalid.json"
    keyring_path = tmp_path / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir()
    keyring_path.touch()
    config = {
        "name": "raspberry-pi-os_armv8",
        "arch": "arm64",
        "triplet": "aarch64-linux-gnu",
        "packages": ["libc6-dev"],
        "repositories": [
            {
                "url": "https://deb.debian.org/debian",
                "suite": "trixie",
                "components": ["main"],
                "signed_by": "keyrings/debian-archive-keyring.gpg",
                "pin_priority": 500,
            },
            {
                # 同じ hostname に別 priority を与えて衝突させる。
                "url": "https://deb.debian.org/debian-security",
                "suite": "trixie-security",
                "components": ["main"],
                "signed_by": "keyrings/debian-archive-keyring.gpg",
                "pin_priority": 700,
            },
        ],
    }
    config_path.write_text(json.dumps(config), encoding="utf-8")

    with pytest.raises(SysrootConfigError):
        load_sysroot_config(config_path)


def test_sysroot_config_fingerprint_is_independent_of_config_path(tmp_path: Path) -> None:
    # 同一内容なら checkout の場所が異なっても同じ sysroot と判定できるようにする。
    first_dir = tmp_path / "first"
    second_dir = tmp_path / "second"
    for config_dir in (first_dir, second_dir):
        (config_dir / "keyrings").mkdir(parents=True)
        (config_dir / "keyrings" / "debian-archive-keyring.gpg").touch()
        write_config(config_dir / "config.json")

    first = load_sysroot_config(first_dir / "config.json")
    second = load_sysroot_config(second_dir / "config.json")

    assert sysroot_config_fingerprint(first) == sysroot_config_fingerprint(second)


def test_sysroot_config_fingerprint_changes_with_pin_priority(tmp_path: Path) -> None:
    # pin_priority を付けたら fingerprint が変わり、新旧設定を確実に区別できることを確認する。
    keyring_path = tmp_path / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir()
    keyring_path.touch()
    plain_path = tmp_path / "plain.json"
    pinned_path = tmp_path / "pinned.json"
    write_config(plain_path)
    raw_config: object = json.loads(plain_path.read_text(encoding="utf-8"))
    if not isinstance(raw_config, dict):
        raise AssertionError("テスト設定は JSON オブジェクトである必要がある")
    raw_config = cast(dict[str, object], raw_config)
    repositories = raw_config.get("repositories")
    if (
        not isinstance(repositories, list)
        or not repositories
        or not isinstance(repositories[0], dict)
    ):
        raise AssertionError("テスト設定の repositories が不正")
    repository = cast(dict[str, object], repositories[0])
    repository["pin_priority"] = 990
    pinned_path.write_text(json.dumps(raw_config), encoding="utf-8")

    plain = load_sysroot_config(plain_path)
    pinned = load_sysroot_config(pinned_path)

    assert sysroot_config_fingerprint(plain) != sysroot_config_fingerprint(pinned)


def test_collect_pin_stanzas_emits_expected_layout(tmp_path: Path) -> None:
    # APT preferences に書き出す stanza が hostname 単位で 1 つずつ、正しい形式で作られること。
    keyring_path = tmp_path / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir()
    keyring_path.touch()
    config_path = tmp_path / "raspberry-pi-os_armv8.json"
    config = {
        "name": "raspberry-pi-os_armv8",
        "arch": "arm64",
        "triplet": "aarch64-linux-gnu",
        "packages": ["libc6-dev", "libcamera-dev"],
        "repositories": [
            {
                "url": "https://deb.debian.org/debian",
                "suite": "trixie",
                "components": ["main"],
                "signed_by": "keyrings/debian-archive-keyring.gpg",
            },
            {
                "url": "https://archive.raspberrypi.com/debian",
                "suite": "trixie",
                "components": ["main"],
                "signed_by": "keyrings/debian-archive-keyring.gpg",
                "pin_priority": 990,
            },
        ],
    }
    config_path.write_text(json.dumps(config), encoding="utf-8")
    loaded = load_sysroot_config(config_path)

    stanzas = _collect_pin_stanzas(loaded)

    # pin を持つ Raspberry Pi ミラーの stanza だけが生成される。Debian ミラー側は pin 未指定で
    # 何も出力されない。
    assert stanzas == [
        'Package: *\nPin: origin "archive.raspberrypi.com"\nPin-Priority: 990',
    ]


def test_fix_absolute_symlinks_makes_existing_target_relative(tmp_path: Path) -> None:
    # sysroot の移動後もリンクがホスト側の /usr/lib を参照しないことを確認する。
    target = tmp_path / "usr" / "lib" / "aarch64-linux-gnu" / "libexample.so.1"
    target.parent.mkdir(parents=True)
    target.touch()
    link = target.parent / "libexample.so"
    link.symlink_to("/usr/lib/aarch64-linux-gnu/libexample.so.1")

    _fix_absolute_symlinks(tmp_path)

    assert link.is_symlink()
    assert not os.readlink(link).startswith("/")
    assert link.resolve() == target


def test_fix_absolute_symlinks_keeps_unresolved_target(tmp_path: Path) -> None:
    # alternatives など展開だけでは解決しないリンクを誤った相対リンクへ変更しない。
    link = tmp_path / "usr" / "bin" / "example"
    link.parent.mkdir(parents=True)
    link.symlink_to("/etc/alternatives/example")

    _fix_absolute_symlinks(tmp_path)

    assert os.readlink(link) == "/etc/alternatives/example"


def test_link_pkgconfig_files_creates_compatibility_links(tmp_path: Path) -> None:
    # WebRTC の pkg-config 探索が従来と同じ場所からターゲット用定義を発見できるようにする。
    source_dir = tmp_path / "usr" / "lib" / "aarch64-linux-gnu" / "pkgconfig"
    source_dir.mkdir(parents=True)
    (source_dir / "example.pc").touch()

    _link_pkgconfig_files(tmp_path, "aarch64-linux-gnu")

    link = tmp_path / "usr" / "share" / "pkgconfig" / "example.pc"
    assert link.is_symlink()
    assert os.readlink(link) == "../../lib/aarch64-linux-gnu/pkgconfig/example.pc"


def test_build_sysroot_reuses_matching_manifest(tmp_path: Path) -> None:
    # 一致する manifest があれば APT を再実行せず、安全に既存 sysroot を再利用する。
    config_path = tmp_path / "config" / "config.json"
    keyring_path = config_path.parent / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir(parents=True)
    keyring_path.touch()
    write_config(config_path)
    config = load_sysroot_config(config_path)
    output_dir = tmp_path / "rootfs"
    output_dir.mkdir()
    manifest = {
        "format_version": 1,
        "fingerprint": sysroot_config_fingerprint(config),
    }
    (output_dir / ".webrtc-build-sysroot.json").write_text(json.dumps(manifest), encoding="utf-8")

    built = build_sysroot(config, output_dir)

    assert built is False


def test_build_sysroot_rejects_old_manifest_without_force(tmp_path: Path) -> None:
    # 生成形式が変わった sysroot を再利用せず、明示的な再生成を要求する。
    config_path = tmp_path / "config" / "config.json"
    keyring_path = config_path.parent / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir(parents=True)
    keyring_path.touch()
    write_config(config_path)
    config = load_sysroot_config(config_path)
    output_dir = tmp_path / "rootfs"
    output_dir.mkdir()
    manifest = {
        "format_version": 0,
        "fingerprint": sysroot_config_fingerprint(config),
    }
    (output_dir / ".webrtc-build-sysroot.json").write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(SysrootBuildError):
        build_sysroot(config, output_dir)


def test_build_sysroot_rejects_stale_directory_without_force(tmp_path: Path) -> None:
    # 設定由来か不明な既存ディレクトリを黙って削除または再利用しない。
    config_path = tmp_path / "config" / "config.json"
    keyring_path = config_path.parent / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir(parents=True)
    keyring_path.touch()
    write_config(config_path)
    config = load_sysroot_config(config_path)
    output_dir = tmp_path / "rootfs"
    output_dir.mkdir()

    with pytest.raises(SysrootBuildError):
        build_sysroot(config, output_dir)


def test_build_sysroot_rejects_stale_symlink_without_force(tmp_path: Path) -> None:
    # 壊れたリンクも既存出力として扱い、利用者の明示なしに置き換えない。
    config_path = tmp_path / "config" / "config.json"
    keyring_path = config_path.parent / "keyrings" / "debian-archive-keyring.gpg"
    keyring_path.parent.mkdir(parents=True)
    keyring_path.touch()
    write_config(config_path)
    config = load_sysroot_config(config_path)
    output_dir = tmp_path / "rootfs"
    output_dir.symlink_to(tmp_path / "missing")

    with pytest.raises(SysrootBuildError):
        build_sysroot(config, output_dir)


def test_load_real_ubuntu_22_04_armv8_jetson_config() -> None:
    # Jetson 用の JSON も実運用と同じ位置から load できることを確認する。
    # スキーマ drift (pin_priority 範囲外、hostname 分割違反、キー綴り違い等) を
    # cross-compile step 起動前に CI で早期に検知するため、単体テストで直接 load する。
    repository_root = Path(__file__).resolve().parent.parent.parent
    config_path = repository_root / "sysroot" / "ubuntu-22.04_armv8_jetson.json"

    config = load_sysroot_config(config_path)

    assert config.name == "ubuntu-22.04_armv8_jetson"
    assert config.arch == "arm64"
    assert config.triplet == "aarch64-linux-gnu"
    # nvidia-jetpack meta-package を入れず、個別に列挙する方針が守られていること。
    assert "nvidia-jetpack" not in config.packages
    assert "nvidia-l4t-jetson-multimedia-api" in config.packages
    # NVIDIA 側は 700、Ubuntu Ports 側は 500 で hostname 単位の pin 優先度が期待通りであること。
    priorities = {
        repository.hostname: repository.pin_priority for repository in config.repositories
    }
    assert priorities == {
        "ports.ubuntu.com": 500,
        "repo.download.nvidia.com": 700,
    }
    # hostname 単位の pin_priority は同一 hostname を潰すため、entry 数と suite の組でも検証する。
    # 将来 t234 side を誤って削除しても、上の priorities assert だけでは検出できない。
    repository_pairs = sorted((r.hostname, r.suite) for r in config.repositories)
    assert repository_pairs == sorted(
        [
            ("ports.ubuntu.com", "jammy"),
            ("repo.download.nvidia.com", "r36.3"),
            ("repo.download.nvidia.com", "r36.3"),
        ]
    )


def test_load_real_raspberry_pi_os_armv8_config() -> None:
    # 実際にリポジトリへ配置した設定と keyring が sysroot_builder の validation を通ることを、
    # ネットワーク無しで確認する。JSON 側の壊れやパス指定ミスを CI で拾える。
    repository_root = Path(__file__).resolve().parent.parent.parent
    config_path = repository_root / "sysroot" / "raspberry-pi-os_armv8.json"

    config = load_sysroot_config(config_path)

    assert config.name == "raspberry-pi-os_armv8"
    assert config.arch == "arm64"
    assert config.triplet == "aarch64-linux-gnu"
    assert "libcamera-dev" in config.packages
    # Raspberry Pi ミラー側だけが pin_priority を持つ想定である。
    priorities = {
        repository.hostname: repository.pin_priority for repository in config.repositories
    }
    assert priorities == {
        "deb.debian.org": None,
        "archive.raspberrypi.com": 990,
    }
