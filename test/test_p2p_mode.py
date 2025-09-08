"""P2P モードの E2E テスト"""

from momo import Momo, MomoMode


def test_with_custom_arguments(free_port, port_allocator):
    """カスタム引数で momo を起動できることを確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
        resolution="QVGA",
        framerate=15,
        log_level="info",
    ) as m:
        data = m.get_metrics()
        assert "version" in data


def test_multiple_instances_concurrent(port_allocator):
    """複数の momo インスタンスを同時に起動できることを確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=next(port_allocator),
        port=next(port_allocator),
        fake_capture_device=True,
    ) as m1:
        with Momo(
            mode=MomoMode.P2P,
            metrics_port=next(port_allocator),
            port=next(port_allocator),
            fake_capture_device=True,
        ) as m2:
            # 両方のインスタンスが正常に動作していることを確認
            data1 = m1.get_metrics()
            data2 = m2.get_metrics()

            assert "version" in data1
            assert "version" in data2


def test_multiple_instances_different_configs(port_allocator):
    """異なる設定で複数のインスタンスを起動できることを確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=next(port_allocator),
        port=next(port_allocator),
        fake_capture_device=True,
        resolution="QVGA",
        framerate=15,
    ) as m1:
        with Momo(
            mode=MomoMode.P2P,
            metrics_port=next(port_allocator),
            port=next(port_allocator),
            fake_capture_device=True,
            resolution="QVGA",
            framerate=15,
        ) as m2:
            # 両方のインスタンスにアクセス可能
            data1 = m1.get_metrics()
            data2 = m2.get_metrics()
            assert "version" in data1
            assert "version" in data2


def test_dynamic_instance_creation_and_cleanup(port_allocator):
    """動的にインスタンスを作成・削除できることを確認"""
    instances = []

    try:
        for i in range(3):
            momo = Momo(
                mode=MomoMode.P2P,
                metrics_port=next(port_allocator),
                port=next(port_allocator),
                fake_capture_device=True,
            )
            momo.__enter__()
            instances.append(momo)

            # 新しく作成したインスタンスが動作していることを確認
            data = momo.get_metrics()
            assert "version" in data

        # すべてのインスタンスが同時に動作していることを確認
        for momo in instances:
            data = momo.get_metrics()
            assert "version" in data

    finally:
        # クリーンアップ
        for momo in instances:
            momo.__exit__(None, None, None)
