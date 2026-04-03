import os

import pytest

from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
# Apple Video Toolbox 環境が有効でない場合もスキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS")
    or not os.environ.get("APPLE_VIDEO_TOOLBOX"),
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
        audio=False,
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

        data = m.get_metrics(
            wait_stats=[
                {"type": "outbound-rtp", "kind": "video", "encoderImplementation": "VideoToolbox"}
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

                    # video の検証
                    if stat["kind"] == "video":
                        assert "framesEncoded" in stat
                        assert "frameWidth" in stat
                        assert "frameHeight" in stat
                        assert stat["framesEncoded"] > 0

                        # エンコーダー実装が VideoToolbox であることを確認
                        assert "encoderImplementation" in stat
                        assert stat["encoderImplementation"] == "VideoToolbox"

                case "codec":
                    # codec の必須フィールドを確認
                    assert "payloadType" in stat
                    assert "mimeType" in stat
                    assert "clockRate" in stat

                    # video codec の検証
                    if stat["mimeType"] == expected_mime_type:
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


@pytest.mark.skip(reason="webrtc-rs が HWA での simulcast に未対応のためスキップ")
@pytest.mark.parametrize(
    "video_codec_type,expected_encoder_implementation",
    [
        ("H264", "VideoToolbox"),
        ("H265", "VideoToolbox"),
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
        audio=False,
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
        # Apple Video Toolbox では SimulcastEncoderAdapter と VideoToolbox の組み合わせ
        assert "SimulcastEncoderAdapter" in outbound_rtp_r0["encoderImplementation"]
        assert "VideoToolbox" in outbound_rtp_r0["encoderImplementation"]

        assert outbound_rtp_r0["frameWidth"] == 240
        assert outbound_rtp_r0["frameHeight"] == 128
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
        assert "SimulcastEncoderAdapter" in outbound_rtp_r1["encoderImplementation"]
        assert "VideoToolbox" in outbound_rtp_r1["encoderImplementation"]

        assert outbound_rtp_r1["frameWidth"] == 480
        assert outbound_rtp_r1["frameHeight"] == 256
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
        assert "SimulcastEncoderAdapter" in outbound_rtp_r2["encoderImplementation"]
        assert "VideoToolbox" in outbound_rtp_r2["encoderImplementation"]

        assert outbound_rtp_r2["frameWidth"] == 960
        assert outbound_rtp_r2["frameHeight"] == 528
        print(f"r2: {outbound_rtp_r2['frameWidth']}x{outbound_rtp_r2['frameHeight']}")

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
        audio=False,
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
            audio=False,
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
                        "encoderImplementation": "VideoToolbox",
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
                        "decoderImplementation": "VideoToolbox",
                    }
                ]
            )
            receiver_stats = receiver_data.get("stats", [])

            # 送信側では outbound-rtp に video が存在することを確認
            sender_outbound_rtp = [
                stat
                for stat in sender_stats
                if stat.get("type") == "outbound-rtp" and stat.get("kind") == "video"
            ]
            assert len(sender_outbound_rtp) >= 1, (
                "Sender should have at least 1 video outbound-rtp stat"
            )

            # 送信側の video codec の mimeType を確認
            sender_codecs = [stat for stat in sender_stats if stat.get("type") == "codec"]
            sender_video_codec = next(
                (stat for stat in sender_codecs if stat.get("mimeType", "").startswith("video/")),
                None,
            )
            assert sender_video_codec is not None, "Video codec should be present"
            assert sender_video_codec["mimeType"] == expected_mime_type, (
                f"Expected {expected_mime_type}, got {sender_video_codec['mimeType']}"
            )

            # 送信側でデータが送信されていることを確認
            for stat in sender_outbound_rtp:
                assert "packetsSent" in stat
                assert "bytesSent" in stat
                assert stat["packetsSent"] > 0
                assert stat["bytesSent"] > 0
                assert "encoderImplementation" in stat
                assert stat["encoderImplementation"] == "VideoToolbox"

            # 受信側では inbound-rtp に video が存在することを確認
            receiver_inbound_rtp = [
                stat
                for stat in receiver_stats
                if stat.get("type") == "inbound-rtp" and stat.get("kind") == "video"
            ]
            assert len(receiver_inbound_rtp) >= 1, (
                "Receiver should have at least 1 video inbound-rtp stat"
            )

            # 受信側の video codec の mimeType を確認
            receiver_codecs = [stat for stat in receiver_stats if stat.get("type") == "codec"]
            receiver_video_codec = next(
                (stat for stat in receiver_codecs if stat.get("mimeType", "").startswith("video/")),
                None,
            )
            assert receiver_video_codec is not None, "Video codec should be present on receiver"
            assert receiver_video_codec["mimeType"] == expected_mime_type, (
                f"Expected {expected_mime_type}, got {receiver_video_codec['mimeType']} on receiver"
            )

            # 受信側でデータが受信されていることを確認
            for stat in receiver_inbound_rtp:
                assert "packetsReceived" in stat
                assert "bytesReceived" in stat
                assert stat["packetsReceived"] > 0
                assert stat["bytesReceived"] > 0
                assert "decoderImplementation" in stat
                assert stat["decoderImplementation"] == "VideoToolbox"
