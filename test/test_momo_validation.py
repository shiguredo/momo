"""Momo のモード固有オプション検証をテストする"""

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
