import os
import time

import pytest
from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS"),
    reason="TEST_SORA_MODE_SIGNALING_URLS not set in environment",
)


@pytest.mark.parametrize(
    "video_codec_type,expected_encoder_implementation",
    [
        ("VP8", "SimulcastEncoderAdapter (libvpx, libvpx, libvpx)"),
        ("VP9", "SimulcastEncoderAdapter (libvpx, libvpx, libvpx)"),
        ("AV1", "SimulcastEncoderAdapter (libaom, libaom, libaom)"),
    ],
)
def test_sora_mode_simulcast(
    http_client, sora_settings, video_codec_type, expected_encoder_implementation, free_port
):
    """Sora モードで接続時の統計情報を確認"""
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
        simulcast=True,
        resolution="960x540",  # 540p の解像度
        video_bit_rate=3000,  # ビットレート 3000
        metadata=sora_settings.metadata,
        log_level="verbose",
        **encoder_params,
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
            stat for stat in stats
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
        expected_mime_type = f"video/{video_codec_type}"
        video_codec_stats = [
            stat for stat in stats
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
            stat for stat in stats
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

        # simulcast では video の outbound-rtp を取得して必ず3本あることを確認
        video_outbound_rtp_stats = [
            stat for stat in stats
            if stat.get("type") == "outbound-rtp" and stat.get("kind") == "video"
        ]
        assert len(video_outbound_rtp_stats) == 3, (
            f"Expected 3 video outbound-rtp for simulcast, but got {len(video_outbound_rtp_stats)}"
        )
        
        # video outbound-rtp の中身を検証
        for video_rtp in video_outbound_rtp_stats:
            assert "ssrc" in video_rtp
            assert "packetsSent" in video_rtp
            assert "bytesSent" in video_rtp
            assert "framesEncoded" in video_rtp
            assert "frameWidth" in video_rtp
            assert "frameHeight" in video_rtp
            assert video_rtp["packetsSent"] > 0
            assert video_rtp["bytesSent"] > 0
            assert video_rtp["framesEncoded"] > 0
            
            # encoder implementation を確認
            assert "encoderImplementation" in video_rtp
            assert video_rtp["encoderImplementation"] == expected_encoder_implementation, (
                f"Expected encoder implementation {expected_encoder_implementation}, "
                f"but got {video_rtp['encoderImplementation']}"
            )

        # 各統計タイプの詳細をチェック（outbound-rtp と codec は上で検証済みなのでスキップ）
        for stat in stats:
            match stat.get("type"):
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
