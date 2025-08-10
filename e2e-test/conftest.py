import json
import os
import platform
import subprocess
import time
import uuid
from pathlib import Path

import httpx
import jwt
import pytest
from dotenv import load_dotenv

# .env ファイルを読み込む
load_dotenv()


@pytest.fixture(scope="session")
def momo_executable():
    """ビルド済みの momo 実行ファイルのパスを返す

    環境変数 MOMO_TARGET を使ってターゲットを指定できる
    例: MOMO_TARGET=macos_arm64 pytest
    """
    project_root = Path(__file__).parent.parent

    # 環境変数からターゲットを取得
    target = os.environ.get("MOMO_TARGET")

    # ターゲットが指定されていない場合は自動検出を試みる
    if not target:
        system = platform.system().lower()
        machine = platform.machine().lower()

        if system == "darwin":
            if machine == "arm64" or machine == "aarch64":
                target = "macos_arm64"
            else:
                target = "macos_x86_64"
        elif system == "linux":
            if machine == "aarch64":
                target = "ubuntu-24.04_armv8"
            else:
                target = "ubuntu-24.04_x86_64"
        else:
            pytest.skip(
                f"Cannot auto-detect target for {system}/{machine}. Please set MOMO_TARGET environment variable."
            )

    # momo のパスを構築（この時点で target は None ではない）
    assert target is not None
    momo_path = project_root / "_build" / target / "release" / "momo" / "momo"

    if not momo_path.exists():
        pytest.skip(
            f"momo executable not found at {momo_path}. Please build with: python3 run.py build {target}"
        )

    return str(momo_path)


@pytest.fixture(scope="function")
def momo_server(momo_executable):
    """momo を --test モードと --metrics-port で起動するフィクスチャ"""
    metrics_port = 9090  # メトリクス用のポート
    port = 8080  # WebSocket/HTTP 用のポート

    # momo を test モードで起動（映像・音声なし）
    process = subprocess.Popen(
        [
            momo_executable,
            "--metrics-port",
            str(metrics_port),
            "--fake-capture-device",
            "test",
            "--port",
            str(port),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # サーバーが起動するまで待機
    with httpx.Client() as client:
        for _ in range(30):  # 最大30秒待機
            try:
                response = client.get(f"http://localhost:{metrics_port}/metrics")
                if response.status_code == 200:
                    break
            except (httpx.ConnectError, httpx.ConnectTimeout):
                time.sleep(1)
        else:
            # エラー時にログを出力
            stdout, stderr = process.communicate(timeout=1)
            process.terminate()
            process.wait(timeout=5)
            pytest.fail(
                f"momo server failed to start within 30 seconds\nstdout: {stdout.decode('utf-8', errors='ignore')}\nstderr: {stderr.decode('utf-8', errors='ignore')}"
            )

    yield metrics_port

    # クリーンアップ
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


@pytest.fixture
def http_client():
    """同期 HTTP クライアントを提供するフィクスチャ"""
    with httpx.Client(timeout=10.0) as client:
        yield client


@pytest.fixture(scope="function")
def sora_server(momo_executable):
    """momo を sora モードで起動するフィクスチャ"""
    # 環境変数から設定を取得
    signaling_urls = os.environ.get("TEST_SIGNALING_URLS")
    channel_id_prefix = os.environ.get("TEST_CHANNEL_ID_PREFIX", "")
    secret = os.environ.get("TEST_SECRET", "")

    if not signaling_urls:
        pytest.skip("TEST_SORA_SIGNALING_URLS is not set in .env file")

    channel_id = f"{channel_id_prefix}{uuid.uuid4().hex[:8]}"

    metadata: dict[str, str] = {}
    if secret:
        payload = {
            "channel_id": channel_id,
            # 現在時刻 + 300 秒 (5分)
            "exp": int(time.time()) + 300,
        }

        access_token = jwt.encode(
            payload,
            secret,
            algorithm="HS256",
        )

        metadata = {"access_token": access_token}

    metrics_port = 9090  # メトリクス用のポート

    # コマンドライン引数を構築
    cmd = [
        momo_executable,
        "--metrics-port",
        str(metrics_port),
        "--fake-capture-device",  # テスト環境でカメラがない場合のため
        "sora",
        "--signaling-urls",
        signaling_urls,
        "--channel-id",
        channel_id,
        "--role",
        "sendonly",  # テスト用にsendonly
        "--audio",
        "true",  # オーディオを有効化
        "--video",
        "true",  # ビデオを有効化
        "--metadata",
        json.dumps(metadata),
    ]

    # momo を sora モードで起動
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # サーバーが起動するまで待機
    with httpx.Client() as client:
        for _ in range(30):  # 最大30秒待機
            try:
                response = client.get(f"http://localhost:{metrics_port}/metrics")
                if response.status_code == 200:
                    break
            except (httpx.ConnectError, httpx.ConnectTimeout):
                time.sleep(1)
        else:
            # エラー時にログを出力
            stdout, stderr = process.communicate(timeout=1)
            process.terminate()
            process.wait(timeout=5)
            pytest.fail(
                f"sora server failed to start within 30 seconds\nstdout: {stdout.decode('utf-8', errors='ignore')}\nstderr: {stderr.decode('utf-8', errors='ignore')}"
            )

    yield metrics_port

    # クリーンアップ
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
