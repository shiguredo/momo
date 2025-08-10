import os

import pytest

# Sora モードのテストは TEST_SORA_SIGNALING_URLS が設定されていない場合スキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SIGNALING_URLS"),
    reason="TEST_SIGNALING_URLS not set in environment",
)


def test_sora_metrics_endpoint_returns_200(sora_server, http_client):
    """Sora モードでメトリクスエンドポイントが 200 を返すことを確認"""
    response = http_client.get(f"http://localhost:{sora_server}/metrics")
    assert response.status_code == 200


def test_sora_metrics_response_structure(sora_server, http_client):
    """Sora モードでメトリクスレスポンスの構造を確認"""
    response = http_client.get(f"http://localhost:{sora_server}/metrics")
    assert response.status_code == 200

    data = response.json()

    # 必須フィールドの確認
    assert "version" in data
    assert "libwebrtc" in data
    assert "environment" in data
    assert "stats" in data

    # バージョン情報が文字列であることを確認
    assert isinstance(data["version"], str)
    assert isinstance(data["libwebrtc"], str)
    assert isinstance(data["environment"], str)


def test_sora_connection_stats(sora_server, http_client):
    """Sora モードで接続時の統計情報を確認"""
    response = http_client.get(f"http://localhost:{sora_server}/metrics")
    assert response.status_code == 200

    data = response.json()
    stats = data["stats"]

    # Sora モードでは接続関連の統計情報が含まれる可能性がある
    assert stats is not None

    # stats が配列の場合、peer-connection 関連の統計を探す
    if isinstance(stats, list) and len(stats) > 0:
        # 統計タイプの確認
        stat_types = {stat.get("type") for stat in stats if "type" in stat}
        print(f"Available stat types: {stat_types}")  # デバッグ用


def test_sora_invalid_endpoint_returns_404(sora_server, http_client):
    """Sora モードで存在しないエンドポイントが 404 を返すことを確認"""
    response = http_client.get(f"http://localhost:{sora_server}/invalid")
    assert response.status_code == 404
