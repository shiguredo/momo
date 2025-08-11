import httpx
import pytest
from dotenv import load_dotenv

# .env ファイルを読み込む
load_dotenv()


@pytest.fixture
def http_client():
    """同期 HTTP クライアントを提供するフィクスチャ"""
    with httpx.Client(timeout=10.0) as client:
        yield client
