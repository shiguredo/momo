"""Ayame モードの E2E テスト"""

import uuid
from typing import Any

import pytest

from momo import Momo, MomoMode

AYAME_SIGNALING_URL = "wss://ayame-labo.shiguredo.app/signaling"


def find_stats(metrics_data: dict[str, Any], **filters) -> dict[str, Any] | None:
    """メトリクスデータから指定した条件に合う最初の統計情報を検索"""
    stats = metrics_data.get("stats", [])
    return next(
        (stat for stat in stats if all(stat.get(key) == value for key, value in filters.items())),
        None,
    )


def find_all_stats(metrics_data: dict[str, Any], **filters) -> list[dict[str, Any]]:
    """メトリクスデータから指定した条件に合う全ての統計情報を検索"""
    stats = metrics_data.get("stats", [])
    return [stat for stat in stats if all(stat.get(key) == value for key, value in filters.items())]


def test_ayame_mode_basic(free_port, port_allocator):
    """Ayame モードで momo を起動できることを確認"""
    room_id = str(uuid.uuid4())

    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        metrics_port=free_port,
        fake_capture_device=True,
        resolution="QVGA",
        framerate=30,
        log_level="info",
    ) as m:
        data = m.get_metrics()
        assert "version" in data


def test_ayame_mode_with_client_id(free_port, port_allocator):
    """Ayame モードで client_id を指定して起動できることを確認"""
    room_id = str(uuid.uuid4())
    client_id = str(uuid.uuid4())

    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        client_id=client_id,
        metrics_port=free_port,
        fake_capture_device=True,
        resolution="QVGA",
        framerate=15,
    ) as m:
        data = m.get_metrics()
        assert "version" in data


def test_ayame_mode_with_video_settings(free_port, port_allocator):
    """Ayame モードでビデオ設定をカスタマイズして起動できることを確認"""
    room_id = str(uuid.uuid4())

    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        metrics_port=free_port,
        fake_capture_device=True,
        resolution="QVGA",
        framerate=15,
        vp8_encoder="software",
    ) as m:
        data = m.get_metrics()
        assert "version" in data


def test_ayame_mode_with_audio_settings(free_port, port_allocator):
    """Ayame モードでオーディオ設定をカスタマイズして起動できることを確認"""
    room_id = str(uuid.uuid4())

    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        metrics_port=free_port,
        fake_capture_device=True,
        disable_echo_cancellation=True,
        disable_auto_gain_control=True,
        disable_noise_suppression=True,
    ) as m:
        data = m.get_metrics()
        assert "version" in data


def test_ayame_mode_with_invalid_codec(port_allocator):
    """存在しないコーデックを指定した場合にエラーで終了することを確認"""
    room_id = str(uuid.uuid4())

    with pytest.raises(RuntimeError, match="momo process exited unexpectedly"):
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url=AYAME_SIGNALING_URL,
            room_id=room_id,
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            ayame_video_codec_type="INVALID_CODEC",  # type: ignore[arg-type]
        ):
            pass

    with pytest.raises(RuntimeError, match="momo process exited unexpectedly"):
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url=AYAME_SIGNALING_URL,
            room_id=room_id,
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            ayame_audio_codec_type="INVALID_AUDIO",  # type: ignore[arg-type]
        ):
            pass
