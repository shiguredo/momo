import os

import pytest

from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
# Raspberry Pi 環境が有効でない場合もスキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS") or not os.environ.get("RASPBERRY_PI"),
    reason="TEST_SORA_MODE_SIGNALING_URLS or RASPBERRY_PI not set in environment",
)


def test_connection_stats(sora_settings, free_port):
    """Sora モードで接続時の統計情報を確認（Raspberry Pi H.264）"""
    video_codec_type = "H264"
    expected_mime_type = "video/H264"

    with Momo(
        fake_capture_device=False,
        use_libcamera=True,
        metrics_port=free_port,
        mode=MomoMode.SORA,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=False,
        video=True,
        video_codec_type=video_codec_type,
        metadata=sora_settings.metadata,
        initial_wait=10,
    ) as m:
        # 接続が確立されるまで待つ
        assert m.wait_for_connection(), (
            f"Failed to establish connection for {video_codec_type} codec"
        )

        data = m.get_metrics()
        stats = data["stats"]

        # Sora モードでは接続関連の統計情報が含まれる
        assert stats is not None
        assert isinstance(stats, list)
        assert len(stats) > 0

        # 統計タイプの収集
        stat_types = {stat.get("type") for stat in stats if "type" in stat}

        # 重要な統計タイプが存在することを確認
        expected_types = {
            "peer-connection",
            "transport",
            "codec",
            "outbound-rtp",
        }
        for expected_type in expected_types:
            assert expected_type in stat_types

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
        video_outbound_rtp = video_outbound_rtp_stats[0]
        assert "ssrc" in video_outbound_rtp
        assert "packetsSent" in video_outbound_rtp
        assert "bytesSent" in video_outbound_rtp
        assert "framesEncoded" in video_outbound_rtp
        assert "frameWidth" in video_outbound_rtp
        assert "frameHeight" in video_outbound_rtp
        assert "encoderImplementation" in video_outbound_rtp
        assert video_outbound_rtp["packetsSent"] > 0
        assert video_outbound_rtp["bytesSent"] > 0
        assert video_outbound_rtp["framesEncoded"] > 0
        # Raspberry Pi では V4L2 H264 エンコーダが使われる
        assert video_outbound_rtp["encoderImplementation"] == "V4L2 H264"

        # transport を取得して確認
        transport_stats = [stat for stat in stats if stat.get("type") == "transport"]
        assert len(transport_stats) == 1, f"Expected 1 transport, but got {len(transport_stats)}"

        # transport の中身を検証
        transport = transport_stats[0]
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
            f"Expected 1 peer-connection, but got {len(peer_connection_stats)}"
        )

        # peer-connection の中身を検証
        peer_connection = peer_connection_stats[0]
        assert "dataChannelsOpened" in peer_connection
