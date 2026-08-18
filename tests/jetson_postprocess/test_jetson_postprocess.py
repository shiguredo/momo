from __future__ import annotations

import os
from pathlib import Path

import pytest

from jetson_postprocess import (
    JetsonPostprocessError,
    fixup_jetson_libnvbuf_fdmap_symlinks,
)

# NVIDIA が配置する libnvbuf_fdmap の実体名（バージョン付き）。
_TARGET_NAME = "libnvbuf_fdmap.so.1.0.0"

# リンカが探す互換 symlink 名。dpkg には登録されていないので後処理で作る。
_LINK_NAME = "libnvbuf_fdmap.so"


def _triplet_dir(rootfs: Path) -> Path:
    # sysroot 内で libnvbuf_fdmap が置かれるパスの共通部分。
    # 実運用の path とテスト fixture を揃えるため helper 化する。
    return rootfs / "usr" / "lib" / "aarch64-linux-gnu"


def _place_target(rootfs: Path, subdir: str) -> Path:
    # NVIDIA パッケージが配置する実体ファイルを fixture として作る。
    target_dir = _triplet_dir(rootfs) / subdir
    target_dir.mkdir(parents=True)
    target = target_dir / _TARGET_NAME
    target.write_bytes(b"dummy nvidia shared object")
    return target


def test_creates_symlink_when_missing_in_nvidia(tmp_path: Path) -> None:
    # JetPack 6 系相当の配置。target 実体はあるが互換 symlink が無い状態から、
    # 相対 symlink が新規に作られることを確認する。
    target = _place_target(tmp_path, "nvidia")

    fixup_jetson_libnvbuf_fdmap_symlinks(tmp_path)

    link = target.parent / _LINK_NAME
    assert link.is_symlink()
    # sysroot を移動しても壊れないよう相対 target で書かれること。
    assert os.readlink(link) == _TARGET_NAME
    assert link.resolve() == target.resolve()


def test_creates_symlink_when_missing_in_tegra(tmp_path: Path) -> None:
    # JetPack 5 系までの配置。tegra 側でも同じ挙動になることを確認する。
    target = _place_target(tmp_path, "tegra")

    fixup_jetson_libnvbuf_fdmap_symlinks(tmp_path)

    link = target.parent / _LINK_NAME
    assert link.is_symlink()
    assert os.readlink(link) == _TARGET_NAME
    assert link.resolve() == target.resolve()


def test_creates_symlink_in_both_subdirs(tmp_path: Path) -> None:
    # tegra と nvidia の両方に target がある混在環境で、両方に symlink が張られることを確認する。
    nvidia_target = _place_target(tmp_path, "nvidia")
    tegra_target = _place_target(tmp_path, "tegra")

    fixup_jetson_libnvbuf_fdmap_symlinks(tmp_path)

    for target in (nvidia_target, tegra_target):
        link = target.parent / _LINK_NAME
        assert link.is_symlink()
        assert os.readlink(link) == _TARGET_NAME


def test_reuses_correct_existing_symlink(tmp_path: Path) -> None:
    # 既に正しい相対 symlink が張られている場合は idempotent に何もしないことを確認する。
    # 2 回連続で呼び出しても症状が変わらない（raise しない）。
    target = _place_target(tmp_path, "nvidia")
    link = target.parent / _LINK_NAME
    link.symlink_to(_TARGET_NAME)

    fixup_jetson_libnvbuf_fdmap_symlinks(tmp_path)
    fixup_jetson_libnvbuf_fdmap_symlinks(tmp_path)

    assert link.is_symlink()
    assert os.readlink(link) == _TARGET_NAME


def test_rejects_symlink_pointing_to_other_target(tmp_path: Path) -> None:
    # 期待と違う target を指す既存 symlink は上書きせず拒否することを確認する。
    _place_target(tmp_path, "nvidia")
    link = _triplet_dir(tmp_path) / "nvidia" / _LINK_NAME
    # 意図的に別のファイルを指す symlink を先に置く。
    other = _triplet_dir(tmp_path) / "nvidia" / "libother.so"
    other.write_bytes(b"other")
    link.symlink_to("libother.so")

    with pytest.raises(JetsonPostprocessError):
        fixup_jetson_libnvbuf_fdmap_symlinks(tmp_path)

    # 拒否時は link を触らず、元の状態が保たれること。
    assert os.readlink(link) == "libother.so"


def test_rejects_dangling_symlink(tmp_path: Path) -> None:
    # 既存 symlink が dangling （target が resolve できない）の場合も拒否することを確認する。
    _place_target(tmp_path, "nvidia")
    link = _triplet_dir(tmp_path) / "nvidia" / _LINK_NAME
    link.symlink_to("does-not-exist.so")

    with pytest.raises(JetsonPostprocessError):
        fixup_jetson_libnvbuf_fdmap_symlinks(tmp_path)

    assert link.is_symlink()
    assert os.readlink(link) == "does-not-exist.so"


def test_rejects_regular_file_at_link_path(tmp_path: Path) -> None:
    # symlink の代わりに通常ファイルが同名で置かれている場合は上書きせず拒否する。
    _place_target(tmp_path, "nvidia")
    link = _triplet_dir(tmp_path) / "nvidia" / _LINK_NAME
    link.write_bytes(b"plain file, must not be overwritten")

    with pytest.raises(JetsonPostprocessError):
        fixup_jetson_libnvbuf_fdmap_symlinks(tmp_path)

    # 拒否時にファイル内容が壊されていないこと。
    assert link.is_file()
    assert not link.is_symlink()
    assert link.read_bytes() == b"plain file, must not be overwritten"


def test_skips_when_target_missing(tmp_path: Path) -> None:
    # target 実体が無い場合は補正の根拠が無いので何もしないことを確認する。
    # ディレクトリ自体は作られていても target が無ければ link を作らない。
    (_triplet_dir(tmp_path) / "nvidia").mkdir(parents=True)

    fixup_jetson_libnvbuf_fdmap_symlinks(tmp_path)

    link = _triplet_dir(tmp_path) / "nvidia" / _LINK_NAME
    assert not link.exists()
    assert not link.is_symlink()


def test_skips_when_rootfs_empty(tmp_path: Path) -> None:
    # sysroot 内に該当ディレクトリが無い場合も安全に何もしないことを確認する。
    # このケースで mkdir 相当を勝手に行うと sysroot の構造を汚す。
    fixup_jetson_libnvbuf_fdmap_symlinks(tmp_path)

    # 予期せぬディレクトリが作られていないこと。
    assert not (_triplet_dir(tmp_path)).exists()
