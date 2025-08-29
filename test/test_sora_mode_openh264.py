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
        expected_encoder_implementation = "SimulcastEncoderAdapter (OpenH264, OpenH264, OpenH264)"
        # 接続が確立されるまで待つ（サイマルキャストの場合は複数の rid を待つ）
        assert m.wait_for_connection(), "Failed to establish simulcast connection with OpenH264"

        data = m.get_metrics(
            wait_stats=[
                {"type": "codec", "mimeType": "video/H264"},
                {
                    "type": "outbound-rtp",
                    "rid": "r0",
                    "encoderImplementation": expected_encoder_implementation,
                },
                {
                    "type": "outbound-rtp",
                    "rid": "r1",
                    "encoderImplementation": expected_encoder_implementation,
                },
                {
                    "type": "outbound-rtp",
                    "rid": "r2",
                    "encoderImplementation": expected_encoder_implementation,
                },
            ],
        )
        stats = data["stats"]

        # サイマルキャストで複数の rid が存在することを確認
        simulcast_stats = [
            stat
            for stat in stats
            if stat.get("type") == "outbound-rtp"
            and stat.get("kind") == "video"
            and stat.get("rid") in ["r0", "r1", "r2"]
            and stat.get("encoderImplementation") == expected_encoder_implementation
        ]
        assert len(simulcast_stats) == 3, (
            f"Expected 3 simulcast streams with OpenH264, but got {len(simulcast_stats)}"
        )

        # 各 rid の統計情報を確認
        rids = {stat.get("rid") for stat in simulcast_stats}
        print(f"Simulcast RIDs with OpenH264: {rids}")

        for simulcast_stat in simulcast_stats:
            assert simulcast_stat["encoderImplementation"] == expected_encoder_implementation
            assert "rid" in simulcast_stat
            assert "ssrc" in simulcast_stat
            assert "packetsSent" in simulcast_stat
            assert "bytesSent" in simulcast_stat
            assert simulcast_stat["packetsSent"] > 0
            assert simulcast_stat["bytesSent"] > 0


def test_sora_mode_openh264_performance_metrics(sora_settings, free_port):
    """OpenH264 エンコーダーのパフォーマンス関連メトリクスを確認"""

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
        resolution="960x540",  # フル HD 解像度でテスト
        framerate=30,
        metadata=sora_settings.metadata,
    ) as m:
        # 接続が磺立され、統計情報が蓄積されるまで待つ
        assert m.wait_for_connection(), (
            "Failed to establish connection for performance metrics test"
        )

        data = m.get_metrics(
            wait_stats=[
                {"type": "codec", "mimeType": "video/H264"},
                {"type": "outbound-rtp", "kind": "video", "encoderImplementation": "OpenH264"},
            ],
            wait_stats_timeout=10,
            wait_after_stats=3,  # 統計情報を蓄積するため追加で待機
        )
        stats = data["stats"]

        # OpenH264 エンコーダーの統計情報を取得
        openh264_stats = [
            stat
            for stat in stats
            if stat.get("type") == "outbound-rtp"
            and stat.get("kind") == "video"
            and stat.get("encoderImplementation") == "OpenH264"
        ]
        assert len(openh264_stats) >= 1, "OpenH264 encoder stats not found"

        openh264_stat = openh264_stats[0]

        # パフォーマンス関連のメトリクスを検証
        assert "totalEncodeTime" in openh264_stat
        assert "framesEncoded" in openh264_stat
        assert "framesSent" in openh264_stat
        assert "frameWidth" in openh264_stat
        assert "frameHeight" in openh264_stat
        assert "framesPerSecond" in openh264_stat
        assert "qualityLimitationReason" in openh264_stat

        # FHD 解像度の確認（960x540）
        assert openh264_stat["frameWidth"] == 960
        assert openh264_stat["frameHeight"] == 528

        # エンコード時間とフレーム数の妥当性を確認
        if openh264_stat["framesEncoded"] > 0:
            avg_encode_time = openh264_stat["totalEncodeTime"] / openh264_stat["framesEncoded"]
            print(f"OpenH264 average encode time per frame: {avg_encode_time:.3f}s")
            # エンコード時間が異常に長くないことを確認（1フレームあたり1秒未満）
            assert avg_encode_time < 1.0, (
                f"Encoding time per frame is too long: {avg_encode_time:.3f}s"
            )
