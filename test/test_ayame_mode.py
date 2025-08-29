"""Ayame モードの E2E テスト"""

import uuid
from typing import Any

import pytest

from momo import Momo, MomoMode

AYAME_SIGNALING_URL = "wss://ayame-labo.shiguredo.app/signaling"


def find_stats(metrics_data: dict[str, Any], **filters) -> dict[str, Any] | None:
    """メトリクスデータから指定した条件に合う最初の統計情報を検索

    Args:
        metrics_data: get_metrics() で取得したメトリクスデータ
        **filters: 検索条件（例: type="outbound-rtp", kind="video"）

    Returns:
        条件に合う統計情報、見つからない場合は None
    """
    stats = metrics_data.get("stats", [])
    return next(
        (stat for stat in stats if all(stat.get(key) == value for key, value in filters.items())),
        None,
    )


def find_all_stats(metrics_data: dict[str, Any], **filters) -> list[dict[str, Any]]:
    """メトリクスデータから指定した条件に合う全ての統計情報を検索

    Args:
        metrics_data: get_metrics() で取得したメトリクスデータ
        **filters: 検索条件（例: type="codec"）

    Returns:
        条件に合う統計情報のリスト
    """
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
        assert "version" in data  # メトリクスが取得できることを確認


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
        assert "version" in data  # メトリクスが取得できることを確認


@pytest.mark.parametrize("codec", ["VP8", "VP9", "AV1"])
def test_ayame_mode_with_codec(port_allocator, codec):
    """Ayame モードで各種コーデックを使用した通信を確認"""
    room_id = str(uuid.uuid4())

    # コーデックごとのエンコーダー/デコーダー設定
    match codec:
        case "VP8":
            codec_settings = {
                "vp8_encoder": "software",
                "vp8_decoder": "software",
            }
        case "VP9":
            codec_settings = {
                "vp9_encoder": "software",
                "vp9_decoder": "software",
            }
        case "AV1":
            codec_settings = {
                "av1_encoder": "software",
                "av1_decoder": "software",
            }
        case _:
            codec_settings = {}

    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        client_id=str(uuid.uuid4()),
        metrics_port=next(port_allocator),
        fake_capture_device=True,
        resolution="QVGA",
        ayame_video_codec_type=codec,  # コーデックを指定
        **codec_settings,
    ) as m1:
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url=AYAME_SIGNALING_URL,
            room_id=room_id,
            client_id=str(uuid.uuid4()),
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            resolution="QVGA",
            ayame_video_codec_type=codec,  # コーデックを指定
            **codec_settings,
        ) as m2:
            # 両方のピアの接続が確立されるまで待機
            assert m1.wait_for_connection(timeout=10), (
                f"M1 failed to establish connection for {codec} codec within timeout"
            )
            assert m2.wait_for_connection(timeout=10), (
                f"M2 failed to establish connection for {codec} codec within timeout"
            )

            p1_data = m1.get_metrics()
            p2_data = m2.get_metrics()

            # p1 の outbound-rtp でコーデックを確認
            assert (
                p1_outbound := find_stats(p1_data, type="outbound-rtp", kind="video")
            ) is not None, "Could not find p1 outbound-rtp video stream"

            assert (codec_id := p1_outbound.get("codecId")) is not None, (
                "No codecId found in p1 outbound-rtp stats"
            )

            # codecId から codec 統計を探す
            assert (p1_codec := find_stats(p1_data, id=codec_id, type="codec")) is not None, (
                f"Could not find codec stats for codecId: {codec_id}"
            )

            mime_type = p1_codec.get("mimeType", "")
            # mimeType から "video/" や "audio/" を除去してコーデック名だけを取得
            codec_name = mime_type.split("/")[-1] if "/" in mime_type else mime_type
            assert codec == codec_name.upper(), f"Expected {codec} codec but got: {mime_type}"
            print(f"P1 codec for {codec}: {mime_type}")

            # p2 の outbound-rtp でもコーデックを確認（双方向通信なので）
            assert (
                p2_outbound := find_stats(p2_data, type="outbound-rtp", kind="video")
            ) is not None, "Could not find p2 outbound-rtp video stream"

            assert (codec_id := p2_outbound.get("codecId")) is not None, (
                "No codecId found in p2 outbound-rtp stats"
            )

            # codecId から codec 統計を探す
            assert (p2_codec := find_stats(p2_data, id=codec_id, type="codec")) is not None, (
                f"Could not find codec stats for codecId: {codec_id}"
            )

            mime_type = p2_codec.get("mimeType", "")
            # mimeType から "video/" や "audio/" を除去してコーデック名だけを取得
            codec_name = mime_type.split("/")[-1] if "/" in mime_type else mime_type
            assert codec == codec_name.upper(), f"Expected {codec} codec but got: {mime_type}"
            print(f"P2 codec for {codec}: {mime_type}")


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
        assert "version" in data  # メトリクスが取得できることを確認


def test_ayame_mode_peer_connection(port_allocator):
    """Ayame モードで2つのピアが双方向通信できることを確認"""
    room_id = str(uuid.uuid4())

    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        client_id=str(uuid.uuid4()),
        metrics_port=next(port_allocator),
        fake_capture_device=True,
        resolution="QVGA",
    ) as m1:
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url=AYAME_SIGNALING_URL,
            room_id=room_id,
            client_id=str(uuid.uuid4()),
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            resolution="QVGA",
        ) as m2:
            # 両方のピアの接続が確立されるまで弅機
            assert m1.wait_for_connection(timeout=10, additional_wait_after_stats=3), (
                "M1 failed to establish connection within timeout"
            )
            assert m2.wait_for_connection(timeout=10, additional_wait_after_stats=3), (
                "M2 failed to establish connection within timeout"
            )

            p1_data = m1.get_metrics()
            p2_data = m2.get_metrics()

            # wait_for_connection が成功している時点で stats は存在しているはず
            # p1 の送受信を確認（送信と受信の両方があるはず）
            assert (
                p1_video_out := find_stats(p1_data, type="outbound-rtp", kind="video")
            ) is not None, "P1 should have video outbound-rtp"
            assert (
                p1_video_in := find_stats(p1_data, type="inbound-rtp", kind="video")
            ) is not None, "P1 should have video inbound-rtp"
            assert p1_video_out.get("packetsSent", 0) > 0
            assert p1_video_in.get("packetsReceived", 0) > 0

            # p2 の送受信を確認（送信と受信の両方があるはず）
            assert (
                p2_video_out := find_stats(p2_data, type="outbound-rtp", kind="video")
            ) is not None, "P2 should have video outbound-rtp"
            assert (
                p2_video_in := find_stats(p2_data, type="inbound-rtp", kind="video")
            ) is not None, "P2 should have video inbound-rtp"
            assert p2_video_out.get("packetsSent", 0) > 0
            assert p2_video_in.get("packetsReceived", 0) > 0

            print(
                f"P1 video - sent: {p1_video_out.get('packetsSent')}, received: {p1_video_in.get('packetsReceived')}"
            )
            print(
                f"P2 video - sent: {p2_video_out.get('packetsSent')}, received: {p2_video_in.get('packetsReceived')}"
            )


def test_ayame_mode_with_invalid_codec(port_allocator):
    """存在しないコーデックを指定した場合にエラーで終了することを確認"""
    room_id = str(uuid.uuid4())

    # 存在しないビデオコーデックを指定
    # 型チェッカーの警告を抑制するため type: ignore コメントを使用
    with pytest.raises(RuntimeError, match="momo process exited unexpectedly"):
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url=AYAME_SIGNALING_URL,
            room_id=room_id,
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            ayame_video_codec_type="INVALID_CODEC",  # type: ignore[arg-type] 存在しないコーデック
        ):
            pass  # ここには到達しないはず

    # 存在しないオーディオコーデックを指定
    with pytest.raises(RuntimeError, match="momo process exited unexpectedly"):
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url=AYAME_SIGNALING_URL,
            room_id=room_id,
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            ayame_audio_codec_type="INVALID_AUDIO",  # type: ignore[arg-type] 存在しないコーデック
        ):
            pass  # ここには到達しないはず
