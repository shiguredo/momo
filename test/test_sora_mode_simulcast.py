import os

import pytest

from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
# macOS では simulcast のパフォーマンスが不足するためスキップ
pytestmark = [
    pytest.mark.skipif(
        not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS"),
        reason="TEST_SORA_MODE_SIGNALING_URLS not set in environment",
    ),
    # pytest.mark.skipif(
    #     platform.system() == "Darwin",
    #     reason="Skipping simulcast test on macOS due to performance limitations",
    # ),
]


@pytest.mark.parametrize(
    "video_codec_type,expected_encoder_implementation",
    [
        ("VP8", "SimulcastEncoderAdapter (libvpx, libvpx, libvpx)"),
        ("VP9", "SimulcastEncoderAdapter (libvpx, libvpx, libvpx)"),
        ("AV1", "SimulcastEncoderAdapter (libaom, libaom, libaom)"),
    ],
)
def test_simulcast(sora_settings, video_codec_type, expected_encoder_implementation, free_port):
    """Sora モードで接続時の統計情報を確認"""
    # デバッグ: テストケースの情報を出力
    print(f"\n=== Running test for {video_codec_type} ===")
    print(f"Expected encoder implementation: {expected_encoder_implementation}")
    print(f"Free port: {free_port}")

    # エンコーダー設定を準備
    encoder_params = {}
    if video_codec_type == "VP8":
        encoder_params["vp8_encoder"] = "software"
        print(f"Setting VP8 encoder parameter: {encoder_params}")
    elif video_codec_type == "VP9":
        encoder_params["vp9_encoder"] = "software"
        print(f"Setting VP9 encoder parameter: {encoder_params}")
    elif video_codec_type == "AV1":
        encoder_params["av1_encoder"] = "software"
        print(f"Setting AV1 encoder parameter: {encoder_params}")

    print(f"Final encoder_params: {encoder_params}")

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
        framerate=15,  # フレームレート 15 fps
        video_bit_rate=3000,  # ビットレート 3000
        metadata=sora_settings.metadata,
        **encoder_params,
    ) as m:
        # 接続が確立されるまで待つ
        assert m.wait_for_connection(), (
            f"Failed to establish simulcast connection for {video_codec_type}"
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

        # r0 の qualityLimitationDurations を出力
        if "qualityLimitationDurations" in outbound_rtp_r0:
            print("r0 qualityLimitationDurations:")
            for reason, duration in outbound_rtp_r0["qualityLimitationDurations"].items():
                print(f"  {reason}: {duration}")

        # r0 のフレームレートを確認（1 fps 以上）
        assert "framesPerSecond" in outbound_rtp_r0
        assert outbound_rtp_r0["framesPerSecond"] >= 1, (
            f"Expected at least 1 fps for r0, but got {outbound_rtp_r0['framesPerSecond']}"
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

        # r1 の qualityLimitationDurations を出力
        if "qualityLimitationDurations" in outbound_rtp_r1:
            print("r1 qualityLimitationDurations:")
            for reason, duration in outbound_rtp_r1["qualityLimitationDurations"].items():
                print(f"  {reason}: {duration}")

        # r1 のフレームレートを確認（1 fps 以上）
        assert "framesPerSecond" in outbound_rtp_r1
        assert outbound_rtp_r1["framesPerSecond"] >= 1, (
            f"Expected at least 1 fps for r1, but got {outbound_rtp_r1['framesPerSecond']}"
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

        # r2 の qualityLimitationDurations を出力
        if "qualityLimitationDurations" in outbound_rtp_r2:
            print("r2 qualityLimitationDurations:")
            for reason, duration in outbound_rtp_r2["qualityLimitationDurations"].items():
                print(f"  {reason}: {duration}")

        # r2 のフレームレートを確認（1 fps 以上）
        assert "framesPerSecond" in outbound_rtp_r2
        assert outbound_rtp_r2["framesPerSecond"] >= 1, (
            f"Expected at least 1 fps for r2, but got {outbound_rtp_r2['framesPerSecond']}"
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
