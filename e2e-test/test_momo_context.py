"""Momo のコンテキストマネージャーの使い方を示すテスト"""

from momo import Momo, MomoMode


def test_with_momo_as_m(http_client):
    """with Momo() as m: の書き方の例"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9098,
        port=8088,
    ) as m:  # m として受け取る
        # m は起動中の Momo インスタンス
        assert m.metrics_port == 9098

        # get_metrics メソッドも使える
        metrics = m.get_metrics(http_client)
        assert "version" in metrics

        # メトリクスポートにアクセス
        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 200


def test_nested_context_managers(http_client):
    """複数の momo インスタンスを入れ子で起動"""
    # 1つ目の momo
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9099,
        port=8089,
    ) as m1:
        # 2つ目の momo（別のインスタンス）
        with Momo(
            mode=MomoMode.TEST,
            metrics_port=9100,
            port=8090,
        ) as m2:
            # 両方のインスタンスにアクセス可能
            assert m1.metrics_port == 9099
            assert m2.metrics_port == 9100

            response1 = http_client.get(f"http://localhost:{m1.metrics_port}/metrics")
            response2 = http_client.get(f"http://localhost:{m2.metrics_port}/metrics")

            assert response1.status_code == 200
            assert response2.status_code == 200


def test_direct_usage_without_fixture(http_client):
    """フィクスチャを使わずに直接 Momo を使う例"""
    # Momo クラスを直接使用（実行ファイルのパスは自動検出）
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9101,
        port=8091,
        resolution="HD",
        framerate=30,
    ) as m:  # 別名で受け取る
        # m は起動中のインスタンス
        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 200

        # get_metrics メソッドも使える
        metrics = m.get_metrics(http_client)
        assert isinstance(metrics, dict)
        assert "version" in metrics
