import os

import pytest

from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
# Intel VPL 環境が有効でない場合もスキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS") or not os.environ.get("INTEL_VPL"),
    reason="TEST_SORA_MODE_SIGNALING_URLS or INTEL_VPL not set in environment",
)


@pytest.mark.parametrize(
    "video_codec_type",
    [
        "VP9",
        "AV1",
        "H264",
        "H265",
    ],
)
def test_connection_stats(sora_settings, video_codec_type, free_port):
    """Sora モードで接続時の統計情報を確認"""
    # expected_mime_type を生成
    expected_mime_type = f"video/{video_codec_type}"

    # エンコーダー設定を準備
    encoder_params = {}
    match video_codec_type:
        case "VP9":
            encoder_params["vp9_encoder"] = "vpl"
        case "AV1":
            encoder_params["av1_encoder"] = "vpl"
        case "H264":
            encoder_params["h264_encoder"] = "vpl"
        case "H265":
            encoder_params["h265_encoder"] = "vpl"

    with Momo(
        fake_capture_device=True,
        metrics_port=free_port,
        mode=MomoMode.SORA,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        video_codec_type=video_codec_type,
        metadata=sora_settings.metadata,
        initial_wait=10,
        **encoder_params,
    ) as m:
        # 接続が確立されるまで待つ
        assert m.wait_for_connection(), (
            f"Failed to establish connection for {video_codec_type} codec"
        )

        data = m.get_metrics()
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
        audio_outbound_rtp = audio_outbound_rtp_stats[0]
        assert "ssrc" in audio_outbound_rtp
        assert "packetsSent" in audio_outbound_rtp
        assert "bytesSent" in audio_outbound_rtp
        assert "headerBytesSent" in audio_outbound_rtp
        assert audio_outbound_rtp["packetsSent"] > 0
        assert audio_outbound_rtp["bytesSent"] > 0
        assert audio_outbound_rtp["headerBytesSent"] > 0

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
        assert video_outbound_rtp["encoderImplementation"] == "libvpl"

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


@pytest.mark.parametrize(
    "video_codec_type,expected_encoder_implementation",
    [
        ("VP9", "SimulcastEncoderAdapter (libvpl, libvpl, libvpl)"),
        ("AV1", "SimulcastEncoderAdapter (libvpl, libvpl, libvpl)"),
        ("H264", "SimulcastEncoderAdapter (libvpl, libvpl, libvpl)"),
        ("H265", "SimulcastEncoderAdapter (libvpl, libvpl, libvpl)"),
    ],
)
def test_simulcast(sora_settings, video_codec_type, expected_encoder_implementation, free_port):
    """Sora モードで simulcast 接続時の統計情報を確認（Intel VPL 使用）"""
    # エンコーダー設定を準備
    encoder_params = {}
    match video_codec_type:
        case "VP9":
            encoder_params["vp9_encoder"] = "vpl"
        case "AV1":
            encoder_params["av1_encoder"] = "vpl"
        case "H264":
            encoder_params["h264_encoder"] = "vpl"
        case "H265":
            encoder_params["h265_encoder"] = "vpl"

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
        initial_wait=10,
        **encoder_params,
    ) as m:
        # 接続が確立されるまで待つ
        assert m.wait_for_connection(), (
            f"Failed to establish connection for {video_codec_type} codec"
        )

        data = m.get_metrics(
            wait_stats=[
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
            wait_after_stats=3,
        )
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
        expected_mime_type = f"video/{video_codec_type}"
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
        audio_outbound_rtp = audio_outbound_rtp_stats[0]
        assert "ssrc" in audio_outbound_rtp
        assert "packetsSent" in audio_outbound_rtp
        assert "bytesSent" in audio_outbound_rtp
        assert "headerBytesSent" in audio_outbound_rtp
        assert audio_outbound_rtp["packetsSent"] > 0
        assert audio_outbound_rtp["bytesSent"] > 0
        assert audio_outbound_rtp["headerBytesSent"] > 0

        # simulcast では video の outbound-rtp を取得して必ず3本あることを確認
        video_outbound_rtp_stats = [
            stat
            for stat in stats
            if stat.get("type") == "outbound-rtp" and stat.get("kind") == "video"
        ]
        assert len(video_outbound_rtp_stats) == 3, (
            f"Expected 3 video outbound-rtp for simulcast, but got {len(video_outbound_rtp_stats)}"
        )

        # rid ごとに分類
        video_outbound_rtp_by_rid = {}
        for video_outbound_rtp in video_outbound_rtp_stats:
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
        assert "ssrc" in outbound_rtp_r0
        assert "rid" in outbound_rtp_r0
        assert outbound_rtp_r0["rid"] == "r0"
        assert "packetsSent" in outbound_rtp_r0
        assert "bytesSent" in outbound_rtp_r0
        assert "framesEncoded" in outbound_rtp_r0
        assert "frameWidth" in outbound_rtp_r0
        assert "frameHeight" in outbound_rtp_r0
        assert outbound_rtp_r0["packetsSent"] > 0
        assert outbound_rtp_r0["bytesSent"] > 0
        assert outbound_rtp_r0["framesEncoded"] > 0

        # r0 の encoder implementation を確認
        assert "encoderImplementation" in outbound_rtp_r0
        assert outbound_rtp_r0["encoderImplementation"] == expected_encoder_implementation, (
            f"Expected encoder implementation {expected_encoder_implementation} for r0, "
            f"but got {outbound_rtp_r0['encoderImplementation']}"
        )

        # r0 の解像度を確認
        # 元の解像度 960x540 の 1/4 スケール (240x135)
        # ただし、エンコーダーが効率化のため高さを 16 の倍数に調整することがある
        # 128 (16×8) または 135 (元の値) の範囲を許容
        assert outbound_rtp_r0["frameWidth"] == 240, (
            f"Expected width 240 for r0, but got {outbound_rtp_r0['frameWidth']}"
        )
        assert 128 <= outbound_rtp_r0["frameHeight"] <= 135, (
            f"Expected height between 128 and 135 for r0, but got {outbound_rtp_r0['frameHeight']}"
        )
        print(f"r0: {outbound_rtp_r0['frameWidth']}x{outbound_rtp_r0['frameHeight']}")

        # r1 (中解像度) の検証
        outbound_rtp_r1 = video_outbound_rtp_by_rid["r1"]
        assert "ssrc" in outbound_rtp_r1
        assert "rid" in outbound_rtp_r1
        assert outbound_rtp_r1["rid"] == "r1"
        assert "packetsSent" in outbound_rtp_r1
        assert "bytesSent" in outbound_rtp_r1
        assert "framesEncoded" in outbound_rtp_r1
        assert "frameWidth" in outbound_rtp_r1
        assert "frameHeight" in outbound_rtp_r1
        assert outbound_rtp_r1["packetsSent"] > 0
        assert outbound_rtp_r1["bytesSent"] > 0
        assert outbound_rtp_r1["framesEncoded"] > 0

        # r1 の encoder implementation を確認
        assert "encoderImplementation" in outbound_rtp_r1
        assert outbound_rtp_r1["encoderImplementation"] == expected_encoder_implementation, (
            f"Expected encoder implementation {expected_encoder_implementation} for r1, "
            f"but got {outbound_rtp_r1['encoderImplementation']}"
        )

        # r1 の解像度を確認
        # 元の解像度 960x540 の 1/2 スケール (480x270)
        # ただし、エンコーダーが効率化のため高さを 16 の倍数に調整することがある
        # 256 (16×16) から 270 (元の値) の範囲を許容
        assert outbound_rtp_r1["frameWidth"] == 480, (
            f"Expected width 480 for r1, but got {outbound_rtp_r1['frameWidth']}"
        )
        assert 256 <= outbound_rtp_r1["frameHeight"] <= 270, (
            f"Expected height between 256 and 270 for r1, but got {outbound_rtp_r1['frameHeight']}"
        )
        print(f"r1: {outbound_rtp_r1['frameWidth']}x{outbound_rtp_r1['frameHeight']}")

        # r2 (高解像度) の検証
        outbound_rtp_r2 = video_outbound_rtp_by_rid["r2"]
        assert "ssrc" in outbound_rtp_r2
        assert "rid" in outbound_rtp_r2
        assert outbound_rtp_r2["rid"] == "r2"
        assert "packetsSent" in outbound_rtp_r2
        assert "bytesSent" in outbound_rtp_r2
        assert "framesEncoded" in outbound_rtp_r2
        assert "frameWidth" in outbound_rtp_r2
        assert "frameHeight" in outbound_rtp_r2
        assert outbound_rtp_r2["packetsSent"] > 0
        assert outbound_rtp_r2["bytesSent"] > 0
        assert outbound_rtp_r2["framesEncoded"] > 0

        # r2 の encoder implementation を確認
        assert "encoderImplementation" in outbound_rtp_r2
        assert outbound_rtp_r2["encoderImplementation"] == expected_encoder_implementation, (
            f"Expected encoder implementation {expected_encoder_implementation} for r2, "
            f"but got {outbound_rtp_r2['encoderImplementation']}"
        )

        # r2 の解像度を確認
        # 元の解像度 960x540 そのまま
        # ただし、エンコーダーが効率化のため高さを 16 の倍数に調整することがある
        # 528 (16×33) から 540 (元の値) の範囲を許容
        assert outbound_rtp_r2["frameWidth"] == 960, (
            f"Expected width 960 for r2, but got {outbound_rtp_r2['frameWidth']}"
        )
        assert 528 <= outbound_rtp_r2["frameHeight"] <= 540, (
            f"Expected height between 528 and 540 for r2, but got {outbound_rtp_r2['frameHeight']}"
        )
        print(f"r2: {outbound_rtp_r2['frameWidth']}x{outbound_rtp_r2['frameHeight']}")

        # パケット数とバイト数の関係を検証（r0 < r1 < r2）
        assert outbound_rtp_r0["bytesSent"] < outbound_rtp_r1["bytesSent"], (
            f"Expected r0 bytesSent ({outbound_rtp_r0['bytesSent']}) < r1 bytesSent ({outbound_rtp_r1['bytesSent']})"
        )
        assert outbound_rtp_r1["bytesSent"] < outbound_rtp_r2["bytesSent"], (
            f"Expected r1 bytesSent ({outbound_rtp_r1['bytesSent']}) < r2 bytesSent ({outbound_rtp_r2['bytesSent']})"
        )

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


@pytest.mark.parametrize(
    "video_codec_type",
    [
        "VP9",
        "AV1",
        "H264",
        "H265",
    ],
)
def test_sora_sendonly_recvonly_pair(
    sora_settings,
    port_allocator,
    video_codec_type,
):
    """Sora モードで sendonly と recvonly のペアを作成して送受信を確認（Intel VPL 使用）"""

    # expected_mime_type を生成
    expected_mime_type = f"video/{video_codec_type}"

    # エンコーダー設定を準備
    encoder_params = {}
    match video_codec_type:
        case "VP9":
            encoder_params["vp9_encoder"] = "vpl"
        case "AV1":
            encoder_params["av1_encoder"] = "vpl"
        case "H264":
            encoder_params["h264_encoder"] = "vpl"
        case "H265":
            encoder_params["h265_encoder"] = "vpl"

    # デコーダー設定を準備
    decoder_params = {}
    match video_codec_type:
        case "VP9":
            decoder_params["vp9_decoder"] = "vpl"
        case "AV1":
            decoder_params["av1_decoder"] = "vpl"
        case "H264":
            decoder_params["h264_decoder"] = "vpl"
        case "H265":
            decoder_params["h265_decoder"] = "vpl"

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
            assert sender.wait_for_connection(), (
                f"Sender failed to establish connection for {video_codec_type}"
            )
            assert receiver.wait_for_connection(), (
                f"Receiver failed to establish connection for {video_codec_type}"
            )

            # 送信側の統計を確認
            sender_data = sender.get_metrics(
                wait_stats=[
                    {
                        "type": "outbound-rtp",
                        "kind": "video",
                        "encoderImplementation": "libvpl",
                    }
                ]
            )
            sender_stats = sender_data.get("stats", [])

            # 受信側の統計を確認
            receiver_data = receiver.get_metrics(
                wait_stats=[
                    {
                        "type": "inbound-rtp",
                        "kind": "video",
                        "decoderImplementation": "libvpl",
                    }
                ]
            )
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

            # 送信側の audio outbound-rtp を取得して確認
            sender_audio_outbound = next(
                (stat for stat in sender_outbound_rtp if stat.get("kind") == "audio"), None
            )
            assert sender_audio_outbound is not None, "Audio outbound-rtp should be present"
            assert "packetsSent" in sender_audio_outbound
            assert "bytesSent" in sender_audio_outbound
            assert sender_audio_outbound["packetsSent"] > 0
            assert sender_audio_outbound["bytesSent"] > 0

            # 送信側の video outbound-rtp を取得して確認
            sender_video_outbound = next(
                (stat for stat in sender_outbound_rtp if stat.get("kind") == "video"), None
            )
            assert sender_video_outbound is not None, "Video outbound-rtp should be present"
            assert "packetsSent" in sender_video_outbound
            assert "bytesSent" in sender_video_outbound
            assert "encoderImplementation" in sender_video_outbound
            assert sender_video_outbound["packetsSent"] > 0
            assert sender_video_outbound["bytesSent"] > 0
            assert sender_video_outbound["encoderImplementation"] == "libvpl"

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

            # 受信側の audio inbound-rtp を取得して確認
            receiver_audio_inbound = next(
                (stat for stat in receiver_inbound_rtp if stat.get("kind") == "audio"), None
            )
            assert receiver_audio_inbound is not None, "Audio inbound-rtp should be present"
            assert "packetsReceived" in receiver_audio_inbound
            assert "bytesReceived" in receiver_audio_inbound
            assert receiver_audio_inbound["packetsReceived"] > 0
            assert receiver_audio_inbound["bytesReceived"] > 0

            # 受信側の video inbound-rtp を取得して確認
            receiver_video_inbound = next(
                (stat for stat in receiver_inbound_rtp if stat.get("kind") == "video"), None
            )
            assert receiver_video_inbound is not None, "Video inbound-rtp should be present"
            assert "packetsReceived" in receiver_video_inbound
            assert "bytesReceived" in receiver_video_inbound
            assert "decoderImplementation" in receiver_video_inbound
            assert receiver_video_inbound["packetsReceived"] > 0
            assert receiver_video_inbound["bytesReceived"] > 0
            assert receiver_video_inbound["decoderImplementation"] == "libvpl"
