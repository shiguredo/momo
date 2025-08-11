"""Momo の柔軟な引数設定をテストする"""

from momo import Momo, MomoMode


def test_momo_with_custom_args(http_client):
    """カスタム引数で momo を起動できることを確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9091,
        port=8081,
        fake_capture_device=True,
    ) as m:
        # メトリクスエンドポイントにアクセスできることを確認
        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 200

        data = response.json()
        assert "version" in data


def test_momo_with_kwargs(http_client):
    """kwargs で直接引数を指定して momo を起動できることを確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9092,
        port=8082,
        fake_capture_device=True,
    ) as m:
        # メトリクスエンドポイントにアクセスできることを確認
        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 200

        data = response.json()
        assert "version" in data


def test_multiple_momo_instances(http_client):
    """複数の momo インスタンスを同時に起動できることを確認"""
    # 1つ目のインスタンス
    with Momo(mode=MomoMode.TEST, metrics_port=9093, port=8083) as m1:
        # 2つ目のインスタンス（別のポートで起動）
        with Momo(mode=MomoMode.TEST, metrics_port=9094, port=8084) as m2:
            # 両方のインスタンスにアクセスできることを確認
            response1 = http_client.get(f"http://localhost:{m1.metrics_port}/metrics")
            assert response1.status_code == 200

            response2 = http_client.get(f"http://localhost:{m2.metrics_port}/metrics")
            assert response2.status_code == 200


def test_get_metrics_method(http_client):
    """get_metrics メソッドが正しく動作することを確認"""
    with Momo(mode=MomoMode.TEST, metrics_port=9096, port=8086) as m:
        # get_metrics メソッドを使用
        metrics = m.get_metrics(http_client)

        assert isinstance(metrics, dict)
        assert "version" in metrics
        assert "libwebrtc" in metrics


def test_direct_instantiation(http_client):
    """Momo クラスを直接インスタンス化できることを確認"""
    # Momo クラスを直接使用
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9095,
        port=8085,
        resolution="VGA",
        framerate=15,
    ) as m:
        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 200

        data = response.json()
        assert "version" in data
        assert "libwebrtc" in data
