import os

import pytest

from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS"),
    reason="TEST_SORA_MODE_SIGNALING_URLS not set in environment",
)


@pytest.mark.parametrize(
    "video_codec_type, expected_encoder_implementation, expected_decoder_implementation",
    [
        ("VP8", "libvpx", "libvpx"),
        ("VP9", "libvpx", "libvpx"),
        ("AV1", "libaom", "dav1d"),
    ],
)
def test_sendonly_recvonly_pair(
    sora_settings,
    port_allocator,
    video_codec_type,
    expected_encoder_implementation,
    expected_decoder_implementation,
):
    """Sora モードで sendonly と recvonly のペアを作成して送受信を確認"""

    print(sora_settings.channel_id)

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

    # デコーダー設定を準備
    decoder_params = {}
    if video_codec_type == "VP8":
        decoder_params["vp8_decoder"] = "software"
    elif video_codec_type == "VP9":
        decoder_params["vp9_decoder"] = "software"
    elif video_codec_type == "AV1":
        decoder_params["av1_decoder"] = "software"

    # 送信専用クライアント
    with Momo(
        mode=MomoMode.SORA,
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
        **encoder_params,
    ) as sender:
        # 受信専用クライアント
        with Momo(
            mode=MomoMode.SORA,
            signaling_urls=sora_settings.signaling_urls,
            channel_id=sora_settings.channel_id,
            role="recvonly",
            metrics_port=next(port_allocator),
            video=True,
            audio=True,
            metadata=sora_settings.metadata,
            **decoder_params,
        ) as receiver:
            # 接続が確立するまで待機
            assert sender.wait_for_connection(
                additional_wait_stats=[
                    {
                        "type": "outbound-rtp",
                        "encoderImplementation": expected_encoder_implementation,
                    }
                ]
            ), f"Sender failed to establish connection for {video_codec_type}"
            assert receiver.wait_for_connection(
                additional_wait_stats=[
                    {
                        "type": "inbound-rtp",
                        "decoderImplementation": expected_decoder_implementation,
                    }
                ]
            ), f"Receiver failed to establish connection for {video_codec_type}"

            # 送信側の統計を確認
            sender_data = sender.get_metrics()
            sender_stats = sender_data.get("stats", [])

            # 受信側の統計を確認
            receiver_data = receiver.get_metrics()
            receiver_stats = receiver_data.get("stats", [])

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

                # video ストリームの場合、encoderImplementation が libvpx であることを確認
                if stat.get("kind") == "video":
                    assert "encoderImplementation" in stat
                    assert stat["encoderImplementation"] == expected_encoder_implementation

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

                # video ストリームの場合、decoderImplementation が libvpx であることを確認
                if stat.get("kind") == "video":
                    assert "decoderImplementation" in stat
                    assert stat["decoderImplementation"] == expected_decoder_implementation


def test_multiple_sendonly_clients(sora_settings, port_allocator):
    """複数の sendonly クライアントが同じチャンネルに接続できることを確認"""

    # 複数の sendonly クライアントを起動
    with Momo(
        mode=MomoMode.SORA,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        metrics_port=next(port_allocator),
        fake_capture_device=True,
        video=True,
        audio=True,
        metadata=sora_settings.metadata,
    ) as sender1:
        with Momo(
            mode=MomoMode.SORA,
            signaling_urls=sora_settings.signaling_urls,
            channel_id=sora_settings.channel_id,
            role="sendonly",
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            video=True,
            audio=True,
            metadata=sora_settings.metadata,
        ) as sender2:
            # 両方のインスタンスが正常に動作していることを確認
            metrics1 = sender1.get_metrics()
            assert metrics1 is not None

            metrics2 = sender2.get_metrics()
            assert metrics2 is not None
