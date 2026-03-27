import os

import pytest

from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS"),
    reason="TEST_SORA_MODE_SIGNALING_URLS not set in environment",
)


def test_sendonly_with_force_nv12_outbound_rtp(sora_settings, free_port):
    """Sora モードで fake capture + --force-nv12 を指定し、送信統計が取得できることを確認"""

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        force_nv12=True,
        metadata=sora_settings.metadata,
    ) as m:
        # 接続が確立するまで待機
        assert m.wait_for_connection(), "Failed to establish connection with --force-nv12"

        # outbound-rtp(kind=video) が出現するまでメトリクスを待つ
        data = m.get_metrics(
            wait_stats=[{"type": "outbound-rtp", "kind": "video"}],
            wait_stats_timeout=10,
        )
        stats = data.get("stats", [])

        # 送信（映像）の統計が存在し、送信が進んでいることを確認
        video_outbound = [
            s for s in stats if s.get("type") == "outbound-rtp" and s.get("kind") == "video"
        ]
        assert len(video_outbound) >= 1, "No video outbound-rtp stats found"
        v = video_outbound[0]
        assert v.get("packetsSent", 0) > 0
        assert v.get("bytesSent", 0) > 0
