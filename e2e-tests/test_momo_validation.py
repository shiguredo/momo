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
            pytest.fail("Should have raised ValueError for sora options in P2P mode")

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
            pytest.fail("Should have raised ValueError for ayame options in P2P mode")

    assert "Invalid options specified for P2P mode" in str(exc_info.value)
    assert "room_id" in str(exc_info.value)
    assert "client_id" in str(exc_info.value)
    assert "ayame" in str(exc_info.value)


def test_sora_mode_with_p2p_options_raises_error():
    """sora モードで p2p モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.SORA,
            signaling_urls="wss://example.com/signaling",
            channel_id="test-channel",
            # これは p2p モード専用のオプション
            document_root="/var/www/html",
        ):
            pytest.fail("Should have raised ValueError for p2p options in Sora mode")

    assert "Invalid options specified for Sora mode" in str(exc_info.value)
    assert "document_root" in str(exc_info.value)
    assert "p2p mode" in str(exc_info.value)


def test_sora_mode_with_ayame_options_raises_error():
    """sora モードで ayame モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.SORA,
            signaling_urls="wss://example.com/signaling",
            channel_id="test-channel",
            # これは ayame モード専用のオプション
            room_id="test-room",
        ):
            pytest.fail("Should have raised ValueError for ayame options in Sora mode")

    assert "Invalid options specified for Sora mode" in str(exc_info.value)
    assert "room_id" in str(exc_info.value)
    assert "ayame mode" in str(exc_info.value)


def test_ayame_mode_with_p2p_options_raises_error():
    """ayame モードで p2p モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url="wss://example.com/signaling",
            room_id="test-room",
            # これは p2p モード専用のオプション
            document_root="/var/www/html",
        ):
            pytest.fail("Should have raised ValueError for p2p options in Ayame mode")

    assert "Invalid options specified for Ayame mode" in str(exc_info.value)
    assert "document_root" in str(exc_info.value)
    assert "p2p" in str(exc_info.value)


def test_ayame_mode_with_sora_options_raises_error():
    """ayame モードで sora モードのオプションを指定するとエラーになることを確認"""
    with pytest.raises(ValueError) as exc_info:
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url="wss://example.com/signaling",
            room_id="test-room",
            # これは sora モード専用のオプション
            role="sendonly",
            simulcast=True,
        ):
            pytest.fail("Should have raised ValueError for sora options in Ayame mode")

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
        resolution="QVGA",
        framerate=15,
        log_level="info",
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
            pytest.fail("Should have raised ValueError for ayame direction in P2P mode")

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
            pytest.fail("Should have raised ValueError for ayame direction in Sora mode")

    assert "Invalid options specified for Sora mode" in str(exc_info.value)
    assert "direction" in str(exc_info.value)
    assert "ayame mode" in str(exc_info.value)
