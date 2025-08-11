"""P2P (Test) モードの E2E テスト"""

from momo import Momo, MomoMode


def test_with_custom_arguments(http_client):
    """カスタム引数で momo を起動できることを確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9097,
        port=8087,
        fake_capture_device=True,
        resolution="HD",
        framerate=30,
        log_level="info",
    ) as m:
        response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 200


def test_multiple_instances_concurrent(http_client):
    """複数の momo インスタンスを同時に起動できることを確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9098,
        port=8088,
        fake_capture_device=True,
    ) as m1:
        with Momo(
            mode=MomoMode.TEST,
            metrics_port=9099,
            port=8089,
            fake_capture_device=True,
        ) as m2:
            # 両方のインスタンスが正常に動作していることを確認
            response1 = http_client.get(f"http://localhost:{m1.metrics_port}/metrics")
            assert response1.status_code == 200

            response2 = http_client.get(f"http://localhost:{m2.metrics_port}/metrics")
            assert response2.status_code == 200

            # それぞれが独立したデータを返すことを確認
            data1 = response1.json()
            data2 = response2.json()

            assert "version" in data1
            assert "version" in data2


def test_multiple_instances_different_configs(http_client):
    """異なる設定で複数のインスタンスを起動できることを確認"""
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9100,
        port=8090,
        fake_capture_device=True,
        resolution="VGA",
        framerate=15,
    ) as m1:
        with Momo(
            mode=MomoMode.TEST,
            metrics_port=9101,
            port=8091,
            fake_capture_device=True,
            resolution="HD",
            framerate=30,
        ) as m2:
            # 両方のインスタンスにアクセス可能
            assert http_client.get(f"http://localhost:{m1.metrics_port}/metrics").status_code == 200
            assert http_client.get(f"http://localhost:{m2.metrics_port}/metrics").status_code == 200


def test_dynamic_instance_creation_and_cleanup(http_client):
    """動的にインスタンスを作成・削除できることを確認"""
    instances = []
    base_metrics_port = 9102
    base_port = 8092

    try:
        for i in range(3):
            momo = Momo(
                mode=MomoMode.TEST,
                metrics_port=base_metrics_port + i,
                port=base_port + i,
                fake_capture_device=True,
            )
            momo.__enter__()
            instances.append(momo)

            # 新しく作成したインスタンスが動作していることを確認
            response = http_client.get(f"http://localhost:{momo.metrics_port}/metrics")
            assert response.status_code == 200

        # すべてのインスタンスが同時に動作していることを確認
        for momo in instances:
            response = http_client.get(f"http://localhost:{momo.metrics_port}/metrics")
            assert response.status_code == 200

    finally:
        # クリーンアップ
        for momo in instances:
            momo.__exit__(None, None, None)