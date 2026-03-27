import os

import pytest

from momo import Momo, MomoMode

# OpenH264 を使用する Sora モードのテストは OPENH264_PATH が設定されていない場合スキップ
pytestmark = [
    pytest.mark.skipif(
        not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS"),
        reason="TEST_SORA_MODE_SIGNALING_URLS not set in environment",
    ),
    pytest.mark.skipif(
        not os.environ.get("OPENH264_PATH"),
        reason="OPENH264_PATH not set in environment",
    ),
]


def test_sora_mode_with_openh264_encoder(sora_settings, free_port):
    """Sora モードで OpenH264 を使用した H.264 エンコーダーが動作することを確認"""

    openh264_path = os.environ.get("OPENH264_PATH")

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        video_codec_type="H264",
        h264_encoder="software",  # OpenH264 はソフトウェアエンコーダー
        openh264=openh264_path,  # 明示的にパスを指定
        metadata=sora_settings.metadata,
    ) as m:
        # 接続が確立されるまで待つ
        assert m.wait_for_connection(), "Failed to establish connection with OpenH264 encoder"

        data = m.get_metrics(
            wait_stats=[
                {"type": "codec", "mimeType": "video/H264"},
                {"type": "outbound-rtp", "kind": "video", "encoderImplementation": "OpenH264"},
            ],
            wait_stats_timeout=10,
        )
        stats = data["stats"]

        # H.264 codec が存在することを確認
        h264_codec_stats = [
            stat
            for stat in stats
            if stat.get("type") == "codec" and stat.get("mimeType") == "video/H264"
        ]
        assert len(h264_codec_stats) >= 1, "H.264 codec not found in stats"

        # video の outbound-rtp で OpenH264 が使用されていることを確認
        video_outbound_rtp_stats = [
            stat
            for stat in stats
            if stat.get("type") == "outbound-rtp"
            and stat.get("kind") == "video"
            and stat.get("encoderImplementation") == "OpenH264"
        ]
        assert len(video_outbound_rtp_stats) >= 1, (
            "OpenH264 encoder not found in outbound-rtp stats"
        )

        # OpenH264 エンコーダーの統計情報を検証
        openh264_rtp = video_outbound_rtp_stats[0]
        assert openh264_rtp["encoderImplementation"] == "OpenH264"
        assert "ssrc" in openh264_rtp
        assert "packetsSent" in openh264_rtp
        assert "bytesSent" in openh264_rtp
        assert "framesEncoded" in openh264_rtp
        assert openh264_rtp["packetsSent"] > 0
        assert openh264_rtp["bytesSent"] > 0
        assert openh264_rtp["framesEncoded"] > 0


def test_sora_mode_with_explicit_openh264_path(sora_settings, free_port):
    """明示的に OpenH264 パスを指定した場合の動作を確認"""

    openh264_path = os.environ.get("OPENH264_PATH")

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        video_codec_type="H264",
        h264_encoder="software",
        openh264=openh264_path,  # 明示的にパスを指定
        metadata=sora_settings.metadata,
    ) as m:
        # 接続が確立されるまで待つ
        assert m.wait_for_connection(), "Failed to establish connection with explicit OpenH264 path"

        data = m.get_metrics(
            wait_stats=[
                {"type": "codec", "mimeType": "video/H264"},
                {"type": "outbound-rtp", "kind": "video", "encoderImplementation": "OpenH264"},
            ],
            wait_stats_timeout=10,
        )
        stats = data["stats"]

        # OpenH264 エンコーダーが使用されていることを確認
        video_outbound_rtp_stats = [
            stat
            for stat in stats
            if stat.get("type") == "outbound-rtp"
            and stat.get("kind") == "video"
            and stat.get("encoderImplementation") == "OpenH264"
        ]
        assert len(video_outbound_rtp_stats) >= 1, "OpenH264 encoder not found with explicit path"


def test_sora_mode_openh264_with_simulcast(sora_settings, free_port):
    """OpenH264 を使用したサイマルキャストの動作を確認"""

    openh264_path = os.environ.get("OPENH264_PATH")

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        video_codec_type="H264",
        video_bit_rate=3000,
        h264_encoder="software",
        openh264=openh264_path,  # 明示的にパスを指定
        simulcast=True,
        resolution="960x540",
        metadata=sora_settings.metadata,
    ) as m:
        # 接続が確立されるまで待つ（サイマルキャストの場合は複数の rid を待つ）
        assert m.wait_for_connection(), "Failed to establish simulcast connection with OpenH264"

        data = m.get_metrics(
            wait_stats=[
                {"type": "codec", "mimeType": "video/H264"},
                {
                    "type": "outbound-rtp",
                    "rid": "r0",
                },
                {
                    "type": "outbound-rtp",
                    "rid": "r1",
                },
                {
                    "type": "outbound-rtp",
                    "rid": "r2",
                },
            ],
            wait_after_stats=3,
        )
        stats = data["stats"]

        # サイマルキャストで複数の rid が存在することを確認
        simulcast_stats = [
            stat
            for stat in stats
            if stat.get("type") == "outbound-rtp"
            and stat.get("kind") == "video"
            and stat.get("rid") in ["r0", "r1", "r2"]
        ]
        assert len(simulcast_stats) == 3, (
            f"Expected 3 simulcast streams with OpenH264, but got {len(simulcast_stats)}"
        )

        # rid ごとに分類
        video_outbound_rtp_by_rid = {}
        for video_outbound_rtp in simulcast_stats:
            rid = video_outbound_rtp.get("rid")
            assert rid in ["r0", "r1", "r2"], f"Unexpected rid: {rid}"
            video_outbound_rtp_by_rid[rid] = video_outbound_rtp

        # 全ての rid が存在することを確認
        assert set(video_outbound_rtp_by_rid.keys()) == {
            "r0",
            "r1",
            "r2",
        }, f"Expected rid r0, r1, r2, but got {set(video_outbound_rtp_by_rid.keys())}"

        # r0 (低解像度) の検証
        outbound_rtp_r0 = video_outbound_rtp_by_rid["r0"]
        assert "encoderImplementation" in outbound_rtp_r0
        # OpenH264 では SimulcastEncoderAdapter と OpenH264 の組み合わせ
        assert "SimulcastEncoderAdapter" in outbound_rtp_r0["encoderImplementation"]
        assert "OpenH264" in outbound_rtp_r0["encoderImplementation"]
        assert "rid" in outbound_rtp_r0
        assert "ssrc" in outbound_rtp_r0
        assert "packetsSent" in outbound_rtp_r0
        assert "bytesSent" in outbound_rtp_r0
        assert "frameWidth" in outbound_rtp_r0
        assert "frameHeight" in outbound_rtp_r0
        assert outbound_rtp_r0["packetsSent"] > 0
        assert outbound_rtp_r0["bytesSent"] > 0
        assert outbound_rtp_r0["frameWidth"] == 240
        assert outbound_rtp_r0["frameHeight"] == 128
        print(f"r0: {outbound_rtp_r0['frameWidth']}x{outbound_rtp_r0['frameHeight']}")

        # r1 (中解像度) の検証
        outbound_rtp_r1 = video_outbound_rtp_by_rid["r1"]
        assert "encoderImplementation" in outbound_rtp_r1
        assert "SimulcastEncoderAdapter" in outbound_rtp_r1["encoderImplementation"]
        assert "OpenH264" in outbound_rtp_r1["encoderImplementation"]
        assert "rid" in outbound_rtp_r1
        assert "ssrc" in outbound_rtp_r1
        assert "packetsSent" in outbound_rtp_r1
        assert "bytesSent" in outbound_rtp_r1
        assert "frameWidth" in outbound_rtp_r1
        assert "frameHeight" in outbound_rtp_r1
        assert outbound_rtp_r1["packetsSent"] > 0
        assert outbound_rtp_r1["bytesSent"] > 0
        assert outbound_rtp_r1["frameWidth"] == 480
        assert outbound_rtp_r1["frameHeight"] == 256
        print(f"r1: {outbound_rtp_r1['frameWidth']}x{outbound_rtp_r1['frameHeight']}")

        # r2 (高解像度) の検証
        outbound_rtp_r2 = video_outbound_rtp_by_rid["r2"]
        assert "encoderImplementation" in outbound_rtp_r2
        assert "SimulcastEncoderAdapter" in outbound_rtp_r2["encoderImplementation"]
        assert "OpenH264" in outbound_rtp_r2["encoderImplementation"]
        assert "rid" in outbound_rtp_r2
        assert "ssrc" in outbound_rtp_r2
        assert "packetsSent" in outbound_rtp_r2
        assert "bytesSent" in outbound_rtp_r2
        assert "frameWidth" in outbound_rtp_r2
        assert "frameHeight" in outbound_rtp_r2
        assert outbound_rtp_r2["packetsSent"] > 0
        assert outbound_rtp_r2["bytesSent"] > 0
        assert outbound_rtp_r2["frameWidth"] == 960
        assert outbound_rtp_r2["frameHeight"] == 528
        print(f"r2: {outbound_rtp_r2['frameWidth']}x{outbound_rtp_r2['frameHeight']}")
