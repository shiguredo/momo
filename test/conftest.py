import itertools
import os
import time
import uuid
from dataclasses import dataclass

import httpx
import jwt
import pytest
from dotenv import load_dotenv

# .env ファイルを読み込む
load_dotenv()


@dataclass
class SoraSettings:
    """Sora モード用の設定"""

    signaling_urls: str
    channel_id_prefix: str
    secret_key: str
    channel_id: str
    metadata: dict


@pytest.fixture
def sora_settings():
    """Sora モード用の設定を提供するフィクスチャ（各テストごとに新しいchannel_idを生成）"""
    # 環境変数から設定を取得（必須）
    signaling_urls = os.environ.get("TEST_SORA_MODE_SIGNALING_URLS")
    if not signaling_urls:
        raise ValueError("TEST_SORA_MODE_SIGNALING_URLS environment variable is required")

    # カンマ区切りをスペース区切りに変換
    if "," in signaling_urls:
        signaling_urls = signaling_urls.replace(",", " ")

    channel_id_prefix = os.environ.get("TEST_SORA_MODE_CHANNEL_ID_PREFIX")
    if not channel_id_prefix:
        raise ValueError("TEST_SORA_MODE_CHANNEL_ID_PREFIX environment variable is required")

    secret_key = os.environ.get("TEST_SORA_MODE_SECRET_KEY")
    if not secret_key:
        raise ValueError("TEST_SORA_MODE_SECRET_KEY environment variable is required")

    # チャンネルIDを生成
    channel_id = f"{channel_id_prefix}{uuid.uuid4().hex[:8]}"

    # メタデータを生成
    payload = {
        "channel_id": channel_id,
        "exp": int(time.time()) + 300,
    }
    access_token = jwt.encode(payload, secret_key, algorithm="HS256")
    metadata = {"access_token": access_token}

    return SoraSettings(
        signaling_urls=signaling_urls,
        channel_id_prefix=channel_id_prefix,
        secret_key=secret_key,
        channel_id=channel_id,
        metadata=metadata,
    )


@pytest.fixture(scope="session")
def port_allocator():
    """セッション全体で共有されるポート番号アロケーター

    50000から始まるポート番号を順番に生成します。
    複数のテストが並列実行されても、各テストに一意のポート番号が割り当てられます。
    """
    return itertools.count(50000)


@pytest.fixture
def free_port(port_allocator):
    """利用可能なポート番号を提供するフィクスチャ

    各テスト関数で使用すると、自動的に一意のポート番号が割り当てられます。
    """
    return next(port_allocator)


@pytest.fixture
def http_client():
    """同期 HTTP クライアントを提供するフィクスチャ"""
    with httpx.Client(timeout=10.0) as client:
        yield client
