import os
import time

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

    # Momo では接続が確立したことを確認する手段がないため、
    # sleep 5 秒で「ほぼ確実に接続しただろう」と仮定する
    time.sleep(5)

    response = http_client.get(f"http://localhost:{sora_server}/metrics")
    assert response.status_code == 200

    data = response.json()
    stats = data["stats"]

    # Sora モードでは接続関連の統計情報が含まれる可能性がある
    assert stats is not None
    assert isinstance(stats, list)
    assert len(stats) > 0

    # 統計タイプの収集
    stat_types = {stat.get("type") for stat in stats if "type" in stat}
    print(f"Available stat types: {stat_types}")

    # 重要な統計タイプが存在することを確認
    expected_types = {
        "peer-connection",
        "transport",
        "codec",
        "outbound-rtp",
    }
    for expected_type in expected_types:
        assert expected_type in stat_types

    # 各統計タイプの詳細をチェック
    for stat in stats:
        match stat.get("type"):
            case "outbound-rtp":
                # outbound-rtp の必須フィールドを確認
                assert "ssrc" in stat
                assert "kind" in stat
                assert "packetsSent" in stat
                assert "bytesSent" in stat

                # データが実際に送信されていることを確認
                assert stat["packetsSent"] > 0
                assert stat["bytesSent"] > 0

                # audio/video の判定
                match stat["kind"]:
                    case "video":
                        assert "framesEncoded" in stat
                        assert "frameWidth" in stat
                        assert "frameHeight" in stat
                        assert stat["framesEncoded"] > 0
                    case "audio":
                        assert "headerBytesSent" in stat
                        assert stat["headerBytesSent"] > 0

            case "codec":
                # codec の必須フィールドを確認
                assert "payloadType" in stat
                assert "mimeType" in stat
                assert "clockRate" in stat

                # codec は audio/opus か video/VP9 のみ許可
                assert stat["mimeType"] in ["audio/opus", "video/VP9"]

                # codec タイプ別の検証
                match stat["mimeType"]:
                    case "audio/opus":
                        assert "channels" in stat
                        assert stat["clockRate"] == 48000
                    case "video/VP9":
                        assert stat["clockRate"] == 90000

            case "transport":
                # transport の必須フィールドを確認
                assert "bytesSent" in stat
                assert "bytesReceived" in stat
                assert "dtlsState" in stat
                assert "iceState" in stat

                # データが実際に送受信されていることを確認
                assert stat["bytesSent"] > 0
                assert stat["bytesReceived"] > 0

                # 接続状態の確認
                assert stat["dtlsState"] == "connected"
                assert stat["iceState"] == "connected"

            case "peer-connection":
                # peer-connection の必須フィールドを確認
                assert "dataChannelsOpened" in stat


def test_sora_invalid_endpoint_returns_404(sora_server, http_client):
    """Sora モードで存在しないエンドポイントが 404 を返すことを確認"""
    response = http_client.get(f"http://localhost:{sora_server}/invalid")
    assert response.status_code == 404
