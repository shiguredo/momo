import itertools

import pytest


@pytest.fixture(scope="session")
def port_allocator():
    """セッション全体で共有されるポート番号アロケーター

    エフェメラルポート開始の 55000 から始まるポート番号を順番に生成します。
    複数のテストが並列実行されても、各テストに一意のポート番号が割り当てられます。
    """
    return itertools.count(56000)


@pytest.fixture
def free_port(port_allocator):
    """利用可能なポート番号を提供するフィクスチャ

    各テスト関数で使用すると、自動的に一意のポート番号が割り当てられます。
    """
    return next(port_allocator)
