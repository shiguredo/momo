"""Momo のモード固有オプション検証をテストする"""

from __future__ import annotations

import subprocess

import pytest

from momo import Momo, MomoMode


def test_p2p_mode_with_sora_options_raises_error():
    """p2p モードで sora モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.P2P,
            # これは sora モード専用のオプション
            signaling_urls="wss://example.com/signaling",
            channel_id="test-channel",
        ):
            pass

    assert "Invalid options specified for P2P mode" in str(exc_info.value)
    assert "signaling_urls" in str(exc_info.value)
    assert "channel_id" in str(exc_info.value)
    assert "sora mode" in str(exc_info.value)


def test_p2p_mode_with_ayame_options_raises_error():
    """p2p モードで ayame モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.P2P,
            # これは ayame モード専用のオプション
            room_id="test-room",
            client_id="test-client",
        ):
            pass

    assert "Invalid options specified for P2P mode" in str(exc_info.value)
    assert "room_id" in str(exc_info.value)
    assert "client_id" in str(exc_info.value)
    assert "ayame" in str(exc_info.value)  # "ayame/sora mode" の可能性もある


def test_sora_mode_with_p2p_options_raises_error():
    """sora モードで p2p モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.SORA,
            signaling_urls="wss://example.com/signaling",  # sora に必要
            channel_id="test-channel",  # sora に必要
            # これは p2p モード専用のオプション
            document_root="/var/www/html",
        ):
            pass

    assert "Invalid options specified for Sora mode" in str(exc_info.value)
    assert "document_root" in str(exc_info.value)
    assert "p2p mode" in str(exc_info.value)


def test_sora_mode_with_ayame_options_raises_error():
    """sora モードで ayame モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.SORA,
            signaling_urls="wss://example.com/signaling",  # sora に必要
            channel_id="test-channel",  # sora に必要
            # これは ayame モード専用のオプション
            room_id="test-room",
        ):
            pass

    assert "Invalid options specified for Sora mode" in str(exc_info.value)
    assert "room_id" in str(exc_info.value)
    assert "ayame mode" in str(exc_info.value)


def test_ayame_mode_with_p2p_options_raises_error():
    """ayame モードで p2p モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url="wss://example.com/signaling",  # ayame に必要
            room_id="test-room",  # ayame に必要
            # これは p2p モード専用のオプション
            document_root="/var/www/html",
        ):
            pass

    assert "Invalid options specified for Ayame mode" in str(exc_info.value)
    assert "document_root" in str(exc_info.value)
    assert "p2p" in str(exc_info.value)  # "p2p/sora mode" の可能性もある


def test_ayame_mode_with_sora_options_raises_error():
    """ayame モードで sora モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url="wss://example.com/signaling",  # ayame に必要
            room_id="test-room",  # ayame に必要
            # これは sora モード専用のオプション
            role="sendonly",
            simulcast=True,
        ):
            pass

    assert "Invalid options specified for Ayame mode" in str(exc_info.value)
    assert "role" in str(exc_info.value)
    assert "simulcast" in str(exc_info.value)
    assert "sora mode" in str(exc_info.value)


def test_common_options_allowed_in_all_modes(free_port, port_allocator):
    """共通オプションはすべてのモードで使用できることを確認"""
    # p2p モードで共通オプションを使用
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
        resolution="QVGA",  # 共通オプション
        framerate=15,  # 共通オプション
        log_level="info",  # 共通オプション
    ) as m:
        data = m.get_metrics()
        assert "version" in data


def test_p2p_mode_with_ayame_direction_raises_error():
    """P2P モードで Ayame の direction オプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.P2P,
            # これは ayame モード専用のオプション
            direction="sendonly",
        ):
            pass

    assert "Invalid options specified for P2P mode" in str(exc_info.value)
    assert "direction" in str(exc_info.value)
    assert "ayame mode" in str(exc_info.value)


def test_sora_mode_with_ayame_direction_raises_error():
    """Sora モードで Ayame の direction オプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.SORA,
            signaling_urls="wss://example.com/signaling",
            channel_id="test-channel",
            # これは ayame モード専用のオプション
            direction="recvonly",
        ):
            pass

    assert "Invalid options specified for Sora mode" in str(exc_info.value)
    assert "direction" in str(exc_info.value)
    assert "ayame mode" in str(exc_info.value)


def test_fake_capture_device_with_no_video_input_device_exits_2() -> None:
    """
    --no-video-input-device と --fake-capture-device の併用は矛盾する。
    momo バイナリがクラッシュせず終了コード 2 と英語エラーを返すことを確認する。
    フェイクキャプチャ非対応ビルドでは当該フラグが無いのでスキップする。
    """
    momo = Momo(
        mode=MomoMode.SORA,
        signaling_urls="wss://example.com",
        channel_id="test",
    )
    help_result = subprocess.run(
        [momo.executable_path, "--help"],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    # フェイクキャプチャ無しのビルドではオプション自体が存在しない
    if "--fake-capture-device" not in help_result.stdout:
        pytest.skip("この momo バイナリは --fake-capture-device を含まない")

    result = subprocess.run(
        [
            momo.executable_path,
            "--no-video-input-device",
            "--fake-capture-device",
            "sora",
            "--signaling-urls",
            "wss://example.com",
            "--channel-id",
            "test",
        ],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert result.returncode == 2
    assert (
        "error: --fake-capture-device cannot be used with --no-video-input-device" in result.stderr
    )
