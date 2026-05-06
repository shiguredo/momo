import os

import pytest

from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS"),
    reason="TEST_SORA_MODE_SIGNALING_URLS not set in environment",
)


@pytest.mark.parametrize(
    "ignore_disconnect_websocket",
    [
        pytest.param(
            "true",
            marks=pytest.mark.xfail(
                strict=True,
                reason=(
                    "sora-rust-sdk が WebSocket の close_notify なし切断を"
                    "致命的エラーとして扱うため、ignore_disconnect_websocket=true 指定時に"
                    "Sora 側からの WebSocket 切断でセッションが終了してしまう。"
                    "sora-rust-sdk 側の connection.rs の UnexpectedEof ハンドリング修正後に"
                    "xfail マークを外す。"
                ),
            ),
        ),
        "false",
    ],
)
def test_sendonly_recvonly_with_ignore_disconnect_websocket(
    sora_settings,
    port_allocator,
    ignore_disconnect_websocket,
):
    """`--ignore-disconnect-websocket` 指定時に sendonly / recvonly ペアが
    接続確立 + RTP 送受信できることを確認する

    `--ignore-disconnect-websocket` は `--data-channel-signaling true` 指定時にのみ
    意味を持つオプションであるため、`--data-channel-signaling true` を併用する。

    本フラグは「WebSocket 切断後も DataChannel 経由でセッションを維持する」
    ことを意図したオプションだが、E2E テストから WebSocket を物理的に切断する
    手段が現状のテスト基盤では用意されていないため、本テストはフラグ指定で
    接続が成立し、RTP 送受信が継続することを検証するリグレッションテストとして
    機能する。

    現状 ``ignore_disconnect_websocket=true`` ケースは sora-rust-sdk 側の
    バグ (close_notify なし切断を致命的エラーとして扱う) により sender 側で
    接続が確立せず失敗するため、xfail マークを付与している。
    sora-rust-sdk 側修正後に xfail マークを外す。
    """

    # 送信専用クライアント
    with Momo(
        mode=MomoMode.SORA,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        metrics_port=next(port_allocator),
        fake_capture_device=True,
        video=True,
        video_codec_type="VP8",
        audio=True,
        vp8_encoder="software",
        data_channel_signaling="true",
        ignore_disconnect_websocket=ignore_disconnect_websocket,
        metadata=sora_settings.metadata,
        initial_wait=10,
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
            vp8_decoder="software",
            data_channel_signaling="true",
            ignore_disconnect_websocket=ignore_disconnect_websocket,
            metadata=sora_settings.metadata,
        ) as receiver:
            # 接続確立を確認
            assert sender.wait_for_connection(), (
                f"Sender failed to establish connection "
                f"(ignore_disconnect_websocket={ignore_disconnect_websocket})"
            )
            assert receiver.wait_for_connection(), (
                f"Receiver failed to establish connection "
                f"(ignore_disconnect_websocket={ignore_disconnect_websocket})"
            )

            # 送信側の統計を取得 (映像エンコーダが libvpx になるまで待機)
            sender_data = sender.get_metrics(
                wait_stats=[
                    {
                        "type": "outbound-rtp",
                        "encoderImplementation": "libvpx",
                    }
                ]
            )
            sender_stats = sender_data.get("stats", [])

            # 受信側の統計を取得 (映像デコーダが libvpx になるまで待機)
            receiver_data = receiver.get_metrics(
                wait_stats=[
                    {
                        "type": "inbound-rtp",
                        "decoderImplementation": "libvpx",
                    }
                ]
            )
            receiver_stats = receiver_data.get("stats", [])

            # 送信側: outbound-rtp が音声と映像の 2 つ存在することを確認
            sender_outbound_rtp = [
                stat for stat in sender_stats if stat.get("type") == "outbound-rtp"
            ]
            assert len(sender_outbound_rtp) == 2, (
                "Sender should have exactly 2 outbound-rtp stats (audio and video)"
            )

            # 送信側: 各ストリームでパケットとバイトが送信されていることを確認
            for stat in sender_outbound_rtp:
                assert stat.get("packetsSent", 0) > 0, (
                    f"Sender outbound-rtp ({stat.get('kind')}) packetsSent should be > 0"
                )
                assert stat.get("bytesSent", 0) > 0, (
                    f"Sender outbound-rtp ({stat.get('kind')}) bytesSent should be > 0"
                )
                if stat.get("kind") == "video":
                    assert stat.get("encoderImplementation") == "libvpx"

            # 受信側: inbound-rtp が音声と映像の 2 つ存在することを確認
            receiver_inbound_rtp = [
                stat for stat in receiver_stats if stat.get("type") == "inbound-rtp"
            ]
            assert len(receiver_inbound_rtp) == 2, (
                "Receiver should have exactly 2 inbound-rtp stats (audio and video)"
            )

            # 受信側: 各ストリームでパケットとバイトが受信されていることを確認
            for stat in receiver_inbound_rtp:
                assert stat.get("packetsReceived", 0) > 0, (
                    f"Receiver inbound-rtp ({stat.get('kind')}) packetsReceived should be > 0"
                )
                assert stat.get("bytesReceived", 0) > 0, (
                    f"Receiver inbound-rtp ({stat.get('kind')}) bytesReceived should be > 0"
                )
                if stat.get("kind") == "video":
                    assert stat.get("decoderImplementation") == "libvpx"
