import os

import pytest
from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
# Apple Video Toolbox 環境が有効でない場合もスキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS") or not os.environ.get("APPLE_VIDEO_TOOLBOX"),
    reason="TEST_SORA_MODE_SIGNALING_URLS or APPLE_VIDEO_TOOLBOX not set in environment",
)


@pytest.mark.parametrize(
    "video_codec_type",
    [
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
    if video_codec_type == "H264":
        encoder_params["h264_encoder"] = "videotoolbox"
    elif video_codec_type == "H265":
        encoder_params["h265_encoder"] = "videotoolbox"

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
        log_level="verbose",
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

                            # エンコーダー実装が VideoToolbox であることを確認
                            assert "encoderImplementation" in stat
                            assert stat["encoderImplementation"] == "VideoToolbox"
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
    "video_codec_type,expected_encoder_implementation",
    [
        ("H264", "SimulcastEncoderAdapter (VideoToolbox, VideoToolbox, VideoToolbox)"),
        ("H265", "SimulcastEncoderAdapter (VideoToolbox, VideoToolbox, VideoToolbox)"),
    ],
)
def test_simulcast(sora_settings, video_codec_type, expected_encoder_implementation, free_port):
    """Sora モードで simulcast 接続時の統計情報を確認（Apple Video Toolbox 使用）"""
    # エンコーダー設定を準備
    encoder_params = {}
    if video_codec_type == "H264":
        encoder_params["h264_encoder"] = "videotoolbox"
    elif video_codec_type == "H265":
        encoder_params["h265_encoder"] = "videotoolbox"

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
        initial_wait=10,
        **encoder_params,
    ) as m:
        # 接続が確立されるまで待つ
        assert m.wait_for_connection(
            additional_wait_stats=[
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
            additional_wait_after_stats=3,
        ), f"Failed to establish connection for {video_codec_type} codec"

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
        assert set(video_outbound_rtp_by_rid.keys()) == {"r0", "r1", "r2"}, (
            f"Expected rid r0, r1, r2, but got {set(video_outbound_rtp_by_rid.keys())}"
        )

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

        # r0 のフレームレートを確認（25 fps 以上）
        assert "framesPerSecond" in outbound_rtp_r0
        assert outbound_rtp_r0["framesPerSecond"] >= 25, (
            f"Expected at least 25 fps for r0, but got {outbound_rtp_r0['framesPerSecond']}"
        )

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

        # r1 のフレームレートを確認（25 fps 以上）
        assert "framesPerSecond" in outbound_rtp_r1
        assert outbound_rtp_r1["framesPerSecond"] >= 25, (
            f"Expected at least 25 fps for r1, but got {outbound_rtp_r1['framesPerSecond']}"
        )

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

        # r2 のフレームレートを確認（25 fps 以上）
        assert "framesPerSecond" in outbound_rtp_r2
        assert outbound_rtp_r2["framesPerSecond"] >= 25, (
            f"Expected at least 25 fps for r2, but got {outbound_rtp_r2['framesPerSecond']}"
        )

        # パケット数とバイト数の関係を検証（r0 < r1 < r2）
        assert outbound_rtp_r0["bytesSent"] < outbound_rtp_r1["bytesSent"], (
            f"Expected r0 bytesSent ({outbound_rtp_r0['bytesSent']}) < r1 bytesSent ({outbound_rtp_r1['bytesSent']})"
        )
        assert outbound_rtp_r1["bytesSent"] < outbound_rtp_r2["bytesSent"], (
            f"Expected r1 bytesSent ({outbound_rtp_r1['bytesSent']}) < r2 bytesSent ({outbound_rtp_r2['bytesSent']})"
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


@pytest.mark.parametrize(
    "video_codec_type",
    [
        "H264",
        "H265",
    ],
)
def test_sora_sendonly_recvonly_pair(
    sora_settings,
    port_allocator,
    video_codec_type,
):
    """Sora モードで sendonly と recvonly のペアを作成して送受信を確認（Apple Video Toolbox 使用）"""

    # expected_mime_type を生成
    expected_mime_type = f"video/{video_codec_type}"

    # エンコーダー設定を準備
    encoder_params = {}
    if video_codec_type == "H264":
        encoder_params["h264_encoder"] = "videotoolbox"
    elif video_codec_type == "H265":
        encoder_params["h265_encoder"] = "videotoolbox"

    # デコーダー設定を準備
    decoder_params = {}
    if video_codec_type == "H264":
        decoder_params["h264_decoder"] = "videotoolbox"
    elif video_codec_type == "H265":
        decoder_params["h265_decoder"] = "videotoolbox"

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

                # video ストリームの場合、encoderImplementation が VideoToolbox であることを確認
                if stat.get("kind") == "video":
                    assert "encoderImplementation" in stat
                    assert stat["encoderImplementation"] == "VideoToolbox"

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

                # video ストリームの場合、decoderImplementation が VideoToolbox であることを確認
                if stat.get("kind") == "video":
                    assert "decoderImplementation" in stat
                    assert stat["decoderImplementation"] == "VideoToolbox"
