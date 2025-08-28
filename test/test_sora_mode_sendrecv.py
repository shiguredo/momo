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
def test_sendrecv(
    sora_settings,
    port_allocator,
    video_codec_type,
    expected_encoder_implementation,
    expected_decoder_implementation,
):
    """Sora モードで sendrecv の双方向通信を確認"""

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

    # クライアント1（sendrecv）
    with Momo(
        mode=MomoMode.SORA,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendrecv",
        metrics_port=next(port_allocator),
        fake_capture_device=True,
        video=True,
        video_codec_type=video_codec_type,
        audio=True,
        metadata=sora_settings.metadata,
        **encoder_params,
        **decoder_params,
    ) as client1:
        # クライアント2（sendrecv）
        with Momo(
            mode=MomoMode.SORA,
            signaling_urls=sora_settings.signaling_urls,
            channel_id=sora_settings.channel_id,
            role="sendrecv",
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            video=True,
            video_codec_type=video_codec_type,
            audio=True,
            metadata=sora_settings.metadata,
            **encoder_params,
            **decoder_params,
        ) as client2:
            # 接続が確立するまで待機
            assert client1.wait_for_connection(
                additional_wait_stats=[
                    {
                        "type": "outbound-rtp",
                        "encoderImplementation": expected_encoder_implementation,
                    },
                    {
                        "type": "inbound-rtp",
                        "decoderImplementation": expected_decoder_implementation,
                    },
                ]
            ), f"Client1 failed to establish connection for {video_codec_type}"
            assert client2.wait_for_connection(
                additional_wait_stats=[
                    {
                        "type": "outbound-rtp",
                        "encoderImplementation": expected_encoder_implementation,
                    },
                    {
                        "type": "inbound-rtp",
                        "decoderImplementation": expected_decoder_implementation,
                    },
                ]
            ), f"Client2 failed to establish connection for {video_codec_type}"

            # クライアント1の統計を確認
            client1_data = client1.get_metrics()
            client1_stats = client1_data.get("stats", [])

            # クライアント2の統計を確認
            client2_data = client2.get_metrics()
            client2_stats = client2_data.get("stats", [])

            # クライアント1: outbound-rtp（送信）が音声と映像の2つ存在することを確認
            client1_outbound_rtp = [
                stat for stat in client1_stats if stat.get("type") == "outbound-rtp"
            ]
            assert (
                len(client1_outbound_rtp) == 2
            ), "Client1 should have exactly 2 outbound-rtp stats (audio and video)"

            # クライアント1: inbound-rtp（受信）が音声と映像の2つ存在することを確認
            client1_inbound_rtp = [
                stat for stat in client1_stats if stat.get("type") == "inbound-rtp"
            ]
            assert (
                len(client1_inbound_rtp) == 2
            ), "Client1 should have exactly 2 inbound-rtp stats (audio and video)"

            # クライアント1の送信データを確認
            for stat in client1_outbound_rtp:
                assert "packetsSent" in stat
                assert "bytesSent" in stat
                assert stat["packetsSent"] > 0
                assert stat["bytesSent"] > 0

                # video ストリームの場合、encoderImplementation を確認
                if stat.get("kind") == "video":
                    assert "encoderImplementation" in stat
                    assert stat["encoderImplementation"] == expected_encoder_implementation

            # クライアント1の受信データを確認
            for stat in client1_inbound_rtp:
                assert "packetsReceived" in stat
                assert "bytesReceived" in stat
                assert stat["packetsReceived"] > 0
                assert stat["bytesReceived"] > 0

                # video ストリームの場合、decoderImplementation を確認
                if stat.get("kind") == "video":
                    assert "decoderImplementation" in stat
                    assert stat["decoderImplementation"] == expected_decoder_implementation

            # クライアント2: outbound-rtp（送信）が音声と映像の2つ存在することを確認
            client2_outbound_rtp = [
                stat for stat in client2_stats if stat.get("type") == "outbound-rtp"
            ]
            assert (
                len(client2_outbound_rtp) == 2
            ), "Client2 should have exactly 2 outbound-rtp stats (audio and video)"

            # クライアント2: inbound-rtp（受信）が音声と映像の2つ存在することを確認
            client2_inbound_rtp = [
                stat for stat in client2_stats if stat.get("type") == "inbound-rtp"
            ]
            assert (
                len(client2_inbound_rtp) == 2
            ), "Client2 should have exactly 2 inbound-rtp stats (audio and video)"

            # クライアント2の送信データを確認
            for stat in client2_outbound_rtp:
                assert "packetsSent" in stat
                assert "bytesSent" in stat
                assert stat["packetsSent"] > 0
                assert stat["bytesSent"] > 0

                # video ストリームの場合、encoderImplementation を確認
                if stat.get("kind") == "video":
                    assert "encoderImplementation" in stat
                    assert stat["encoderImplementation"] == expected_encoder_implementation

            # クライアント2の受信データを確認
            for stat in client2_inbound_rtp:
                assert "packetsReceived" in stat
                assert "bytesReceived" in stat
                assert stat["packetsReceived"] > 0
                assert stat["bytesReceived"] > 0

                # video ストリームの場合、decoderImplementation を確認
                if stat.get("kind") == "video":
                    assert "decoderImplementation" in stat
                    assert stat["decoderImplementation"] == expected_decoder_implementation

            # コーデック情報の確認（両クライアント）
            for client_stats, client_name in [(client1_stats, "Client1"), (client2_stats, "Client2")]:
                codecs = [stat for stat in client_stats if stat.get("type") == "codec"]
                assert len(codecs) >= 2, f"{client_name} should have at least 2 codecs (audio and video)"

                # video codec の mimeType を確認
                video_codec = next(
                    (stat for stat in codecs if stat.get("mimeType", "").startswith("video/")),
                    None,
                )
                assert video_codec is not None, f"Video codec should be present on {client_name}"
                assert (
                    video_codec["mimeType"] == expected_mime_type
                ), f"Expected {expected_mime_type}, got {video_codec['mimeType']} on {client_name}"

                # audio codec の mimeType を確認
                audio_codec = next(
                    (stat for stat in codecs if stat.get("mimeType", "").startswith("audio/")),
                    None,
                )
                assert audio_codec is not None, f"Audio codec should be present on {client_name}"
                assert (
                    audio_codec["mimeType"] == "audio/opus"
                ), f"Audio codec should be opus on {client_name}"