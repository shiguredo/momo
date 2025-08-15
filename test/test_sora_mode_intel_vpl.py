import os
import time

import pytest
from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
# Intel VPL 環境が有効でない場合もスキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS") or not os.environ.get("INTEL_VPL"),
    reason="TEST_SORA_MODE_SIGNALING_URLS or INTEL_VPL not set in environment",
)


@pytest.mark.parametrize(
    "video_codec_type,expected_mime_type",
    [
        ("VP9", "video/VP9"),
        ("AV1", "video/AV1"),
        ("H264", "video/H264"),
        ("H265", "video/H265"),
    ],
)
def test_sora_connection_stats(
    http_client, sora_settings, video_codec_type, expected_mime_type, free_port
):
    """Sora モードで接続時の統計情報を確認"""
    with Momo(
        fake_capture_device=True,
        vp9_encoder="vpl",
        av1_encoder="vpl",
        h264_encoder="vpl",
        h265_encoder="vpl",
        metrics_port=free_port,
        mode=MomoMode.SORA,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        video_codec_type=video_codec_type,
        metadata=sora_settings.metadata,
        log_level="verbose",
    ) as m:
        time.sleep(3)

        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 200

        data = response.json()
        stats = data["stats"]

        # Sora モードでは接続関連の統計情報が含まれる可能性がある
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

        # 指定されたビデオコーデックが実際に使われていることを確認
        codec_mime_types = {
            stat.get("mimeType")
            for stat in stats
            if stat.get("type") == "codec" and "mimeType" in stat
        }
        assert expected_mime_type in codec_mime_types

        # 各統計タイプの詳細をチェック
        for stat in stats:
            match stat.get("type"):
                case "outbound-rtp":
                    # outbound-rtp の必須フィールドを確認
                    assert "ssrc" in stat
                    assert "kind" in stat
                    assert "packetsSent" in stat
                    assert "bytesSent" in stat

                    # データが実際に送信されていることを確認
                    assert stat["packetsSent"] > 0
                    assert stat["bytesSent"] > 0

                    # audio/video の判定
                    match stat["kind"]:
                        case "video":
                            assert "framesEncoded" in stat
                            assert "frameWidth" in stat
                            assert "frameHeight" in stat
                            assert stat["framesEncoded"] > 0

                            # エンコーダー実装が Intel VPL であることを確認
                            assert "encoderImplementation" in stat
                            assert stat["encoderImplementation"] == "libvpl"
                        case "audio":
                            assert "headerBytesSent" in stat
                            assert stat["headerBytesSent"] > 0

                case "codec":
                    # codec の必須フィールドを確認
                    assert "payloadType" in stat
                    assert "mimeType" in stat
                    assert "clockRate" in stat

                    # codec は audio/opus か指定されたビデオコーデックのみ許可
                    assert stat["mimeType"] in ["audio/opus", expected_mime_type]

                    # codec タイプ別の検証
                    if stat["mimeType"] == "audio/opus":
                        assert "channels" in stat
                        assert stat["clockRate"] == 48000
                    elif stat["mimeType"] == expected_mime_type:
                        assert stat["clockRate"] == 90000

                case "transport":
                    # transport の必須フィールドを確認
                    assert "bytesSent" in stat
                    assert "bytesReceived" in stat
                    assert "dtlsState" in stat
                    assert "iceState" in stat

                    # データが実際に送受信されていることを確認
                    assert stat["bytesSent"] > 0
                    assert stat["bytesReceived"] > 0

                    # 接続状態の確認
                    assert stat["dtlsState"] == "connected"
                    assert stat["iceState"] == "connected"

                case "peer-connection":
                    # peer-connection の必須フィールドを確認
                    assert "dataChannelsOpened" in stat


@pytest.mark.parametrize(
    "video_codec_type, expected_mime_type",
    [
        ("VP9", "video/VP9"),
        ("AV1", "video/AV1"),
        ("H264", "video/H264"),
        ("H265", "video/H265"),
    ],
)
def test_sora_sendonly_recvonly_pair(
    http_client,
    sora_settings,
    port_allocator,
    video_codec_type,
    expected_mime_type,
):
    """Sora モードで sendonly と recvonly のペアを作成して送受信を確認（Intel VPL 使用）"""

    # 送信専用クライアント
    with Momo(
        mode=MomoMode.SORA,
        vp9_encoder="vpl",
        vp9_decoder="vpl",
        av1_encoder="vpl",
        av1_decoder="vpl",
        h264_encoder="vpl",
        h264_decoder="vpl",
        h265_encoder="vpl",
        h265_decoder="vpl",
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        metrics_port=next(port_allocator),
        fake_capture_device=True,
        video=True,
        video_codec_type=video_codec_type,
        audio=True,
        metadata=sora_settings.metadata,
        initial_wait=10,
    ) as sender:
        # 受信専用クライアント
        with Momo(
            mode=MomoMode.SORA,
            vp9_decoder="vpl",
            av1_decoder="vpl",
            h264_decoder="vpl",
            h265_decoder="vpl",
            signaling_urls=sora_settings.signaling_urls,
            channel_id=sora_settings.channel_id,
            role="recvonly",
            metrics_port=next(port_allocator),
            video=True,
            audio=True,
            metadata=sora_settings.metadata,
        ) as receiver:
            # 接続が確立するまで待機
            time.sleep(5)
            
            # 送信側の統計を確認
            sender_response = http_client.get(f"http://localhost:{sender.metrics_port}/metrics")
            sender_stats = sender_response.json().get("stats", [])

            # 受信側の統計を確認
            receiver_response = http_client.get(f"http://localhost:{receiver.metrics_port}/metrics")
            receiver_stats = receiver_response.json().get("stats", [])

            # 送信側では outbound-rtp が音声と映像の2つ存在することを確認
            sender_outbound_rtp = [
                stat for stat in sender_stats if stat.get("type") == "outbound-rtp"
            ]
            assert len(sender_outbound_rtp) == 2, (
                "Sender should have exactly 2 outbound-rtp stats (audio and video)"
            )

            # 送信側の codec 情報を確認（音声と映像で少なくとも2つ）
            sender_codecs = [stat for stat in sender_stats if stat.get("type") == "codec"]
            assert len(sender_codecs) >= 2, "Should have at least 2 codecs (audio and video)"

            # video codec の mimeType を確認
            sender_video_codec = next(
                (stat for stat in sender_codecs if stat.get("mimeType", "").startswith("video/")),
                None,
            )
            assert sender_video_codec is not None, "Video codec should be present"
            assert sender_video_codec["mimeType"] == expected_mime_type, (
                f"Expected {expected_mime_type}, got {sender_video_codec['mimeType']}"
            )

            # audio codec の mimeType を確認
            sender_audio_codec = next(
                (stat for stat in sender_codecs if stat.get("mimeType", "").startswith("audio/")),
                None,
            )
            assert sender_audio_codec is not None, "Audio codec should be present"
            assert sender_audio_codec["mimeType"] == "audio/opus", "Audio codec should be opus"

            # 送信側でデータが送信されていることを確認
            for stat in sender_outbound_rtp:
                assert "packetsSent" in stat
                assert "bytesSent" in stat
                assert stat["packetsSent"] > 0
                assert stat["bytesSent"] > 0
                
                # video ストリームの場合、encoderImplementation が libvpl であることを確認
                if stat.get("kind") == "video":
                    assert "encoderImplementation" in stat
                    assert stat["encoderImplementation"] == "libvpl"

            # 受信側では inbound-rtp が音声と映像の2つ存在することを確認
            receiver_inbound_rtp = [
                stat for stat in receiver_stats if stat.get("type") == "inbound-rtp"
            ]
            assert len(receiver_inbound_rtp) == 2, (
                "Receiver should have exactly 2 inbound-rtp stats (audio and video)"
            )

            # 受信側の codec 情報を確認（音声と映像で少なくとも2つ）
            receiver_codecs = [stat for stat in receiver_stats if stat.get("type") == "codec"]
            assert len(receiver_codecs) >= 2, (
                "Should have at least 2 codecs (audio and video) on receiver"
            )

            # video codec の mimeType を確認
            receiver_video_codec = next(
                (stat for stat in receiver_codecs if stat.get("mimeType", "").startswith("video/")),
                None,
            )
            assert receiver_video_codec is not None, "Video codec should be present on receiver"
            assert receiver_video_codec["mimeType"] == expected_mime_type, (
                f"Expected {expected_mime_type}, got {receiver_video_codec['mimeType']} on receiver"
            )

            # audio codec の mimeType を確認
            receiver_audio_codec = next(
                (stat for stat in receiver_codecs if stat.get("mimeType", "").startswith("audio/")),
                None,
            )
            assert receiver_audio_codec is not None, "Audio codec should be present on receiver"
            assert receiver_audio_codec["mimeType"] == "audio/opus", (
                "Audio codec should be opus on receiver"
            )

            # 受信側でデータが受信されていることを確認
            for stat in receiver_inbound_rtp:
                assert "packetsReceived" in stat
                assert "bytesReceived" in stat
                assert stat["packetsReceived"] > 0
                assert stat["bytesReceived"] > 0
                
                # video ストリームの場合、decoderImplementation が libvpl であることを確認
                if stat.get("kind") == "video":
                    assert "decoderImplementation" in stat
                    assert stat["decoderImplementation"] == "libvpl"
