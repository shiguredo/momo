"""メトリクス API の E2E テスト"""

from momo import Momo, MomoMode


def test_metrics_endpoint_returns_200(http_client):
    """メトリクスエンドポイントが 200 を返すことを確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9090,
        port=8080,
        fake_capture_device=True,
    ) as m:
        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 200


def test_metrics_endpoint_returns_json(http_client):
    """メトリクスエンドポイントが JSON を返すことを確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9091,
        port=8081,
        fake_capture_device=True,
    ) as m:
        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 200
        assert response.headers.get("content-type", "").startswith("application/json")

        data = response.json()
        assert isinstance(data, dict)


def test_metrics_response_structure(http_client):
    """メトリクスレスポンスの構造を確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9092,
        port=8082,
        fake_capture_device=True,
    ) as m:
        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
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

        # stats フィールドが存在することを確認（初期状態では空配列の可能性）
        assert data["stats"] is not None


def test_metrics_stats_format(http_client):
    """統計情報の形式を確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9093,
        port=8083,
        fake_capture_device=True,
    ) as m:
        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 200

        data = response.json()
        stats = data["stats"]

        # stats は配列またはオブジェクトであるべき
        assert isinstance(stats, (list, dict))

        # W3C WebRTC Stats の形式に準拠していることを確認
        # 初期状態では空の可能性があるため、内容があればチェック
        if isinstance(stats, list) and len(stats) > 0:
            # 各統計エントリに必須フィールドがあることを確認
            for stat in stats:
                assert "id" in stat
                assert "type" in stat
                assert "timestamp" in stat


def test_invalid_endpoint_returns_404(http_client):
    """存在しないエンドポイントが 404 を返すことを確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9094,
        port=8084,
        fake_capture_device=True,
    ) as m:
        response = http_client.get(f"http://localhost:{m.metrics_port}/invalid")
        assert response.status_code == 404


def test_post_method_returns_error(http_client):
    """POST メソッドがエラーを返すことを確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9095,
        port=8085,
        fake_capture_device=True,
    ) as m:
        response = http_client.post(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 400  # Bad Request


def test_get_metrics_method(http_client):
    """get_metrics メソッドが正しく動作することを確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9096,
        port=8086,
    ) as m:
        # get_metrics メソッドを使用
        metrics = m.get_metrics(http_client)

        assert isinstance(metrics, dict)
        assert "version" in metrics
        assert "libwebrtc" in metrics