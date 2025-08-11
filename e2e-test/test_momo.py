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


def test_multiple_instances_concurrent(http_client):
    """複数の momo インスタンスを同時に起動できることを確認"""
    # 2つのインスタンスを同時に起動
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9096,
        port=8086,
        fake_capture_device=True,
    ) as m1:
        with Momo(
            mode=MomoMode.TEST,
            metrics_port=9097,
            port=8087,
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
    # 異なる解像度設定で複数インスタンスを起動
    with Momo(
        mode=MomoMode.TEST,
        metrics_port=9098,
        port=8088,
        fake_capture_device=True,
        resolution="VGA",
        framerate=15,
    ) as m1:
        with Momo(
            mode=MomoMode.TEST,
            metrics_port=9099,
            port=8089,
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
    base_metrics_port = 9100
    base_port = 8090

    try:
        # 3つのインスタンスを順次作成
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
