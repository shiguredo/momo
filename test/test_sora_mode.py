import os

import pytest
from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS"),
    reason="TEST_SORA_MODE_SIGNALING_URLS not set in environment",
)


def test_metrics_endpoint_returns_200(sora_settings, free_port):
    """Sora モードでメトリクスエンドポイントが 200 を返すことを確認"""

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        metadata=sora_settings.metadata,
    ) as m:
        data = m.get_metrics()
        assert data is not None  # メトリクスが取得できることを確認


def test_metrics_endpoint_response_structure(sora_settings, free_port):
    """Sora モードでメトリクスレスポンスの構造を確認"""

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        metadata=sora_settings.metadata,
    ) as m:
        data = m.get_metrics()

        # 必須フィールドの確認
        assert "version" in data
        assert "libwebrtc" in data
        assert "environment" in data
        assert "stats" in data

        # バージョン情報が文字列であることを確認
        assert isinstance(data["version"], str)
        assert isinstance(data["libwebrtc"], str)
        assert isinstance(data["environment"], str)


def test_invalid_metrics_endpoint_returns_404(sora_settings, free_port):
    """Sora モードで存在しないエンドポイントが 404 を返すことを確認"""

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        metadata=sora_settings.metadata,
    ) as m:
        response = m._http_client.get(f"http://localhost:{m.metrics_port}/invalid")
        assert response.status_code == 404
