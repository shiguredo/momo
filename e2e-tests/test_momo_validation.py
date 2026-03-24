"""Momo のモード固有オプション検証をテストする"""

import pytest

from momo import Momo, MomoMode


def test_p2p_mode_with_ayame_options_raises_error():
    """p2p モードで ayame モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.P2P,
            room_id="test-room",
            client_id="test-client",
        ):
            pass

    assert "Invalid options specified for P2P mode" in str(exc_info.value)
    assert "room_id" in str(exc_info.value)
    assert "client_id" in str(exc_info.value)
    assert "ayame" in str(exc_info.value)


def test_ayame_mode_with_p2p_options_raises_error():
    """ayame モードで p2p モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url="wss://example.com/signaling",
            room_id="test-room",
            document_root="/var/www/html",
        ):
            pass

    assert "Invalid options specified for Ayame mode" in str(exc_info.value)
    assert "document_root" in str(exc_info.value)
    assert "p2p" in str(exc_info.value)


def test_p2p_mode_with_ayame_direction_raises_error():
    """P2P モードで Ayame の direction オプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.P2P,
            direction="sendonly",
        ):
            pass

    assert "Invalid options specified for P2P mode" in str(exc_info.value)
    assert "direction" in str(exc_info.value)
    assert "ayame mode" in str(exc_info.value)


def test_common_options_allowed_in_all_modes(free_port, port_allocator):
    """共通オプションはすべてのモードで使用できることを確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
        resolution="QVGA",
        framerate=15,
        log_level="info",
    ) as m:
        data = m.get_metrics()
        assert "version" in data
