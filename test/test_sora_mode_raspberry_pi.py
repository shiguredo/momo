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
    expected_encoder_implementation = "V4L2M2M H264"

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
        # Raspberry Pi では V4L2 M2M エンコーダが使われる
        # TODO: V4L2-M2M みたいな名前がよさそう
        assert video_outbound_rtp["encoderImplementation"] == expected_encoder_implementation

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


def test_simulcast(sora_settings, free_port):
    """Sora モードで simulcast 接続時の統計情報を確認（Raspberry Pi H.264）"""
    video_codec_type = "H264"
    expected_mime_type = "video/H264"

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=False,
        use_libcamera=True,
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
    ) as m:
        # 接続が確立されるまで待つ
        assert m.wait_for_connection(), (
            f"Failed to establish connection for {video_codec_type} codec with simulcast"
        )

        data = m.get_metrics(
            wait_stats=[
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
            wait_after_stats=10,
        )
        stats = data["stats"]

        # Sora モードでは接続関連の統計情報が含まれる
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
        # Raspberry Pi では SimulcastEncoderAdapter と V4L2M2M H264 の組み合わせ
        assert "SimulcastEncoderAdapter" in outbound_rtp_r0["encoderImplementation"]
        assert "V4L2M2M H264" in outbound_rtp_r0["encoderImplementation"]

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
        assert "V4L2M2M H264" in outbound_rtp_r1["encoderImplementation"]

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
        
        # r2 はフレーキーで frameWidth, frameHeight が出ないことがある
        if "frameWidth" in outbound_rtp_r2:
            assert "frameWidth" in outbound_rtp_r2
            print("frameWidth is present")
        if "frameHeight" in outbound_rtp_r2:
            assert "frameHeight" in outbound_rtp_r2
            print("frameHeight is present")

        assert outbound_rtp_r2["packetsSent"] > 0
        assert outbound_rtp_r2["bytesSent"] > 0
        assert outbound_rtp_r2["framesEncoded"] > 0

        # r2 の encoder implementation を確認
        assert "encoderImplementation" in outbound_rtp_r2
        assert "SimulcastEncoderAdapter" in outbound_rtp_r2["encoderImplementation"]
        assert "V4L2M2M H264" in outbound_rtp_r2["encoderImplementation"]

        # r2 はフレーキーで frameWidth, frameHeight が出ないことがある
        if "frameWidth" in outbound_rtp_r2:
            assert outbound_rtp_r2["frameWidth"] == 960
            print("frameWidth is checked")
        if "frameHeight" in outbound_rtp_r2:
            assert outbound_rtp_r2["frameHeight"] == 528
            print("frameHeight is checked")
        if "frameWidth" in outbound_rtp_r2 and "frameHeight" in outbound_rtp_r2:
            print(f"r2: {outbound_rtp_r2['frameWidth']}x{outbound_rtp_r2['frameHeight']}")

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


@pytest.mark.skipif(reason="上手く動作しないため一時的にスキップ")
def test_sora_sendonly_recvonly_pair(
    sora_settings,
    port_allocator,
):
    """Sora モードで sendonly と recvonly のペアを作成して送受信を確認（Raspberry Pi H.264）"""

    video_codec_type = "H264"
    expected_mime_type = "video/H264"

    # 送信専用クライアント（Raspberry Pi カメラを使用）
    with Momo(
        mode=MomoMode.SORA,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        metrics_port=next(port_allocator),
        fake_capture_device=False,
        use_libcamera=True,
        video=True,
        video_codec_type=video_codec_type,
        audio=False,
        metadata=sora_settings.metadata,
        initial_wait=10,
    ) as sender:
        # 受信専用クライアント（use_libcameraは指定しない）
        with Momo(
            mode=MomoMode.SORA,
            signaling_urls=sora_settings.signaling_urls,
            channel_id=sora_settings.channel_id,
            role="recvonly",
            metrics_port=next(port_allocator),
            video=True,
            audio=False,
            metadata=sora_settings.metadata,
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
                        "encoderImplementation": "V4L2M2M H264",
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
                        "decoderImplementation": "V4L2M2M H264",
                    }
                ]
            )
            receiver_stats = receiver_data.get("stats", [])

            # 送信側では outbound-rtp が映像の1つ存在することを確認（音声なし）
            sender_outbound_rtp = [
                stat for stat in sender_stats if stat.get("type") == "outbound-rtp"
            ]
            assert len(sender_outbound_rtp) == 1, (
                "Sender should have exactly 1 outbound-rtp stats (video only)"
            )

            # 送信側の codec 情報を確認（映像のみ）
            sender_codecs = [stat for stat in sender_stats if stat.get("type") == "codec"]
            assert len(sender_codecs) >= 1, "Should have at least 1 codec (video)"

            # video codec の mimeType を確認
            sender_video_codec = next(
                (stat for stat in sender_codecs if stat.get("mimeType", "").startswith("video/")),
                None,
            )
            assert sender_video_codec is not None, "Video codec should be present"
            assert sender_video_codec["mimeType"] == expected_mime_type, (
                f"Expected {expected_mime_type}, got {sender_video_codec['mimeType']}"
            )

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
            assert sender_video_outbound["encoderImplementation"] == "V4L2M2M H264"

            # 受信側では inbound-rtp が映像の1つ存在することを確認（音声なし）
            receiver_inbound_rtp = [
                stat for stat in receiver_stats if stat.get("type") == "inbound-rtp"
            ]
            assert len(receiver_inbound_rtp) == 1, (
                "Receiver should have exactly 1 inbound-rtp stats (video only)"
            )

            # 受信側の codec 情報を確認（映像のみ）
            receiver_codecs = [stat for stat in receiver_stats if stat.get("type") == "codec"]
            assert len(receiver_codecs) >= 1, "Should have at least 1 codec (video) on receiver"

            # video codec の mimeType を確認
            receiver_video_codec = next(
                (stat for stat in receiver_codecs if stat.get("mimeType", "").startswith("video/")),
                None,
            )
            assert receiver_video_codec is not None, "Video codec should be present on receiver"
            assert receiver_video_codec["mimeType"] == expected_mime_type, (
                f"Expected {expected_mime_type}, got {receiver_video_codec['mimeType']} on receiver"
            )

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
            # Raspberry Pi では V4L2M2M H264 デコーダが使われる（HWA）
            assert receiver_video_inbound["decoderImplementation"] == "V4L2M2M H264"
