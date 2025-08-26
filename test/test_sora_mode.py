import os
import time

import pytest
from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS"),
    reason="TEST_SORA_MODE_SIGNALING_URLS not set in environment",
)


def test_metrics_endpoint_returns_200(sora_settings, free_port):
    """Sora モードでメトリクスエンドポイントが 200 を返すことを確認"""

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        metadata=sora_settings.metadata,
    ) as m:
        data = m.get_metrics()
        assert data is not None  # メトリクスが取得できることを確認


def test_metrics_endpoint_response_structure(sora_settings, free_port):
    """Sora モードでメトリクスレスポンスの構造を確認"""

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        metadata=sora_settings.metadata,
    ) as m:
        data = m.get_metrics()

        # 必須フィールドの確認
        assert "version" in data
        assert "libwebrtc" in data
        assert "environment" in data
        assert "stats" in data

        # バージョン情報が文字列であることを確認
        assert isinstance(data["version"], str)
        assert isinstance(data["libwebrtc"], str)
        assert isinstance(data["environment"], str)


@pytest.mark.parametrize(
    "video_codec_type",
    [
        "VP8",
        "VP9",
        "AV1",
    ],
)
def test_metrics_endpoint(sora_settings, video_codec_type, free_port):
    """Sora モードで接続時の統計情報を確認"""
    # expected_mime_type を生成
    expected_mime_type = f"video/{video_codec_type}"

    # エンコーダー設定を準備
    encoder_params = {}
    if video_codec_type == "VP8":
        encoder_params["vp8_encoder"] = "software"
    elif video_codec_type == "VP9":
        encoder_params["vp9_encoder"] = "software"
    elif video_codec_type == "AV1":
        encoder_params["av1_encoder"] = "software"

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        video_codec_type=video_codec_type,
        metadata=sora_settings.metadata,
        log_level="verbose",
        **encoder_params,
    ) as m:
        # 接続が確立されるまで待つ
        assert m.wait_for_connection(timeout=10), \
            f"Failed to establish connection for {video_codec_type} codec"

        data = m.get_metrics()
        stats = data["stats"]

        # Sora モードでは接続関連の統計情報が含まれる可能性がある
        assert stats is not None
        assert isinstance(stats, list)
        assert len(stats) > 0

        # 統計タイプの収集
        stat_types = {stat.get("type") for stat in stats if "type" in stat}
        print(f"Available stat types: {stat_types}")

        # 重要な統計タイプが存在することを確認
        expected_types = {
            "peer-connection",
            "transport",
            "codec",
            "outbound-rtp",
        }
        for expected_type in expected_types:
            assert expected_type in stat_types

        # audio codec を取得して確認
        audio_codec_stats = [
            stat
            for stat in stats
            if stat.get("type") == "codec" and stat.get("mimeType") == "audio/opus"
        ]
        assert len(audio_codec_stats) == 1, (
            f"Expected 1 audio codec (opus), but got {len(audio_codec_stats)}"
        )

        # audio codec の中身を検証
        audio_codec = audio_codec_stats[0]
        assert "payloadType" in audio_codec
        assert "mimeType" in audio_codec
        assert "clockRate" in audio_codec
        assert "channels" in audio_codec
        assert audio_codec["clockRate"] == 48000

        # video codec を取得して確認
        video_codec_stats = [
            stat
            for stat in stats
            if stat.get("type") == "codec" and stat.get("mimeType") == expected_mime_type
        ]
        assert len(video_codec_stats) == 1, (
            f"Expected 1 video codec ({expected_mime_type}), but got {len(video_codec_stats)}"
        )

        # video codec の中身を検証
        video_codec = video_codec_stats[0]
        assert "payloadType" in video_codec
        assert "mimeType" in video_codec
        assert "clockRate" in video_codec
        assert video_codec["clockRate"] == 90000

        # audio の outbound-rtp を取得して確認
        audio_outbound_rtp_stats = [
            stat
            for stat in stats
            if stat.get("type") == "outbound-rtp" and stat.get("kind") == "audio"
        ]
        assert len(audio_outbound_rtp_stats) == 1, (
            f"Expected 1 audio outbound-rtp, but got {len(audio_outbound_rtp_stats)}"
        )

        # audio outbound-rtp の中身を検証
        audio_rtp = audio_outbound_rtp_stats[0]
        assert "ssrc" in audio_rtp
        assert "packetsSent" in audio_rtp
        assert "bytesSent" in audio_rtp
        assert "headerBytesSent" in audio_rtp
        assert audio_rtp["packetsSent"] > 0
        assert audio_rtp["bytesSent"] > 0
        assert audio_rtp["headerBytesSent"] > 0

        # video の outbound-rtp を取得して確認
        video_outbound_rtp_stats = [
            stat
            for stat in stats
            if stat.get("type") == "outbound-rtp" and stat.get("kind") == "video"
        ]
        assert len(video_outbound_rtp_stats) == 1, (
            f"Expected 1 video outbound-rtp, but got {len(video_outbound_rtp_stats)}"
        )

        # video outbound-rtp の中身を検証
        video_rtp = video_outbound_rtp_stats[0]
        assert "ssrc" in video_rtp
        assert "packetsSent" in video_rtp
        assert "bytesSent" in video_rtp
        assert "framesEncoded" in video_rtp
        assert "frameWidth" in video_rtp
        assert "frameHeight" in video_rtp
        assert video_rtp["packetsSent"] > 0
        assert video_rtp["bytesSent"] > 0
        assert video_rtp["framesEncoded"] > 0

        # transport を取得して確認
        transport_stats = [stat for stat in stats if stat.get("type") == "transport"]
        assert len(transport_stats) >= 1, (
            f"Expected at least 1 transport stat, but got {len(transport_stats)}"
        )

        # transport の中身を検証
        for transport in transport_stats:
            assert "bytesSent" in transport
            assert "bytesReceived" in transport
            assert "dtlsState" in transport
            assert "iceState" in transport
            assert transport["bytesSent"] > 0
            assert transport["bytesReceived"] > 0
            assert transport["dtlsState"] == "connected"
            assert transport["iceState"] == "connected"

        # peer-connection を取得して確認
        peer_connection_stats = [stat for stat in stats if stat.get("type") == "peer-connection"]
        assert len(peer_connection_stats) == 1, (
            f"Expected 1 peer-connection stat, but got {len(peer_connection_stats)}"
        )

        # peer-connection の中身を検証
        peer_connection = peer_connection_stats[0]
        assert "dataChannelsOpened" in peer_connection


def test_invalid_metrics_endpoint_returns_404(sora_settings, free_port):
    """Sora モードで存在しないエンドポイントが 404 を返すことを確認"""

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        metadata=sora_settings.metadata,
    ) as m:
        response = m._http_client.get(f"http://localhost:{m.metrics_port}/invalid")
        assert response.status_code == 404
