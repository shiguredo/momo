"""Jetson 用 sysroot の後処理モジュール。

sysroot_builder.py は sora-python-sdk canonical と byte-identical に保つため、
Jetson 固有の後処理はこのモジュールに独立して閉じ込める。run.py の
install_sysroot() 直後に呼び出す前提。

libnvbuf_fdmap.so.1.0.0 は sysroot 設定 JSON で明示指定する
nvidia-l4t-multimedia-utils から供給される想定（JetPack 6 系 / r36.x で確認）。
JSON の packages から nvidia-l4t-multimedia-utils を外すと、後処理は補正の
根拠が無いと判定して silent skip し、リンク時に -lnvbuf_fdmap が解決できず
CI の代表パス確認 step 「Verify Jetson sysroot contents」で発火する。
"""

from __future__ import annotations

import logging
import os
from pathlib import Path

__all__ = ["JetsonPostprocessError", "fixup_jetson_libnvbuf_fdmap_symlinks"]


# NVIDIA が配置する libnvbuf_fdmap の実体名（バージョン付き）。
_LIBNVBUF_FDMAP_TARGET_NAME = "libnvbuf_fdmap.so.1.0.0"

# リンカが解決するために必要な互換 symlink 名。dpkg には登録されていない。
_LIBNVBUF_FDMAP_LINK_NAME = "libnvbuf_fdmap.so"

# JetPack 5 系までは tegra、JetPack 6 系からは nvidia に配置される。
# どちらの配置でも動くよう両方を対象にする。
_LIBNVBUF_FDMAP_SUBDIRS = ("tegra", "nvidia")


class JetsonPostprocessError(RuntimeError):
    """Jetson 後処理が安全に続行できない状況を検出したときに送出する。"""


def fixup_jetson_libnvbuf_fdmap_symlinks(rootfs_dir: str | os.PathLike[str]) -> None:
    """sysroot に libnvbuf_fdmap.so の互換 symlink を作成する。

    NVIDIA が配置する libnvbuf_fdmap.so.1.0.0 に対して、リンカが探す
    libnvbuf_fdmap.so の相対 symlink を張り直す。tegra / nvidia の
    両ディレクトリを対象にする（JetPack 5 系までは tegra、
    JetPack 6 系からは nvidia）。

    - target 実体（libnvbuf_fdmap.so.1.0.0）が無いディレクトリは何もしない。
    - 既に正しい symlink が張られている場合は idempotent に何もしない。
    - 既存 symlink が別 target を指す、dangling である、または通常ファイル
      として存在する場合は JetsonPostprocessError で拒否する（意図しない
      上書きで sysroot が壊れるのを防ぐ）。
    """
    root = Path(rootfs_dir)
    triplet_dir = root / "usr" / "lib" / "aarch64-linux-gnu"
    for subdir in _LIBNVBUF_FDMAP_SUBDIRS:
        target = triplet_dir / subdir / _LIBNVBUF_FDMAP_TARGET_NAME
        link = triplet_dir / subdir / _LIBNVBUF_FDMAP_LINK_NAME
        # target 実体が無いディレクトリは補正の根拠がないので飛ばす。
        # is_file は broken symlink には False を返すため dangling も同時に除外できる。
        if not target.is_file():
            continue
        _ensure_relative_symlink(link, target.name)


def _ensure_relative_symlink(link: Path, expected_target_name: str) -> None:
    """link が expected_target_name を指す相対 symlink になるようにする。

    canonical の sysroot_builder.py に手を入れずに Jetson 特有の後処理を
    足せるよう、上書き契約はここに閉じ込める。
    """
    if link.is_symlink():
        current = os.readlink(link)
        # 呼び出し元の入口ガードで target 実体の存在は保証されているため、
        # target 名が一致する既存 symlink は必ず resolve できる。
        # 一致しない場合は自動修復すると意図せぬ上書きになるため拒否する。
        if current == expected_target_name:
            return
        raise JetsonPostprocessError(
            f"Unexpected existing symlink at {link}: current target is {current!r}, "
            f"expected {expected_target_name!r}"
        )
    if link.exists():
        # 通常ファイルが同名で存在する場合は上書きせず拒否する。
        raise JetsonPostprocessError(
            f"Refusing to replace an existing regular file with a symlink: {link}"
        )
    logging.info("Creating Jetson symlink: %s -> %s", link, expected_target_name)
    link.symlink_to(expected_target_name)
