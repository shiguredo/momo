"""TLS 証明書検証（信頼ストア分岐と TURN-TLS）の E2E テスト"""

import os
import subprocess
import time
from pathlib import Path

import jwt
import pytest

from momo import Momo, MomoMode

# Sora モードのテストは TEST_SORA_MODE_SIGNALING_URLS が設定されていない場合スキップ
pytestmark = pytest.mark.skipif(
    not os.environ.get("TEST_SORA_MODE_SIGNALING_URLS"),
    reason="TEST_SORA_MODE_SIGNALING_URLS not set in environment",
)

# Let's Encrypt が公開している ISRG Root X1（公開物。秘密情報ではない）
ISRG_ROOT_X1_PEM = Path(__file__).parent / "data" / "isrgrootx1.pem"


def _create_self_signed_ca_pem(directory: Path) -> Path:
    """openssl で一時的な自己発行 CA の PEM を生成する。

    Sora Labo のサーバ証明書はこの CA では検証できないため、
    `--ca-cert` に指定すると証明書検証失敗になる。
    """
    ca_pem = directory / "self_signed_ca.pem"
    key_pem = directory / "self_signed_ca.key"
    subprocess.run(
        [
            "openssl",
            "req",
            "-x509",
            "-newkey",
            "rsa:2048",
            "-keyout",
            str(key_pem),
            "-out",
            str(ca_pem),
            "-days",
            "1",
            "-nodes",
            "-subj",
            "/CN=momo-e2e-tls-test-ca",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return ca_pem


def _resolve_ca_cert_path(ca_cert_kind: str, tmp_path: Path) -> str | None:
    """テストケース名から `--ca-cert` に渡すパスを決める。未指定は None。"""
    if ca_cert_kind == "system":
        return None
    if ca_cert_kind == "isrg_root_x1":
        assert ISRG_ROOT_X1_PEM.is_file(), f"ISRG Root X1 PEM が見つからない: {ISRG_ROOT_X1_PEM}"
        return str(ISRG_ROOT_X1_PEM)
    if ca_cert_kind == "self_signed":
        return str(_create_self_signed_ca_pem(tmp_path))
    raise ValueError(f"unknown ca_cert_kind: {ca_cert_kind}")


def _metadata_with_turn_tls_only(sora_settings) -> dict[str, str]:
    """Sora Labo 向けに JWT プライベートクレーム turn_tls_only を付けた metadata を作る。

    metadata 直下への turn_tls_only 指定は Sora Labo では廃止済みのため、
    access_token (JWT) のクレームに載せる。共通の sora_settings fixture は壊さない。
    """
    payload = {
        "channel_id": sora_settings.channel_id,
        "exp": int(time.time()) + 300,
        "turn_tls_only": True,
    }
    access_token = jwt.encode(payload, sora_settings.secret_key, algorithm="HS256")
    return {"access_token": access_token}


@pytest.mark.parametrize(
    "ca_cert_kind, insecure",
    [
        ("system", False),
        ("isrg_root_x1", False),
        ("self_signed", True),
    ],
    ids=["system-ca", "ca-cert-isrg-root-x1", "self-signed-insecure"],
)
def test_sora_tls_trust_store_success(
    sora_settings,
    free_port,
    tmp_path,
    ca_cert_kind,
    insecure,
):
    """
    Sora 通常接続で信頼ストア分岐の成功を確認する。

    前提:
    - 未指定は OS のシステム CA
    - `--ca-cert` に ISRG Root X1 を渡すとその PEM のみを trust anchor にする
    - 自己発行 CA + `--insecure` は検証をスキップして接続できる

    期待:
    - `Momo` プロセスの起動に成功し、`wait_for_connection` が成功する
    """
    ca_cert = _resolve_ca_cert_path(ca_cert_kind, tmp_path)
    print(
        f"信頼ストア成功ケース: ca_cert_kind={ca_cert_kind}, insecure={insecure}, ca_cert={ca_cert}"
    )

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        metadata=sora_settings.metadata,
        ca_cert=ca_cert,
        insecure=insecure,
    ) as m:
        assert m.wait_for_connection(timeout=20), (
            f"接続が確立しなかった (ca_cert_kind={ca_cert_kind}, insecure={insecure})"
        )


def test_sora_self_signed_ca_without_insecure_fails_startup(
    sora_settings,
    free_port,
    tmp_path,
):
    """
    自己発行 CA のみ（`--insecure` なし）では Sora 通常接続に失敗することを確認する。

    前提:
    - `--ca-cert` に自己発行 CA だけを渡すとサーバ証明書を検証できない
    - TLS 失敗時もプロセスは即終了せず再接続する
    - Sora では `_wait_for_startup` が metrics の `stats` に要素が入るまで待つため、接続が確立しないとタイムアウトする

    期待:
    - `Momo` プロセスの起動が `_wait_for_startup` 由来の RuntimeError で失敗する
    """
    ca_cert = str(_create_self_signed_ca_pem(tmp_path))
    print(f"信頼ストア失敗ケース: ca_cert={ca_cert}")

    with (
        pytest.raises(RuntimeError, match="failed to start within"),
        Momo(
            mode=MomoMode.SORA,
            metrics_port=free_port,
            fake_capture_device=True,
            signaling_urls=sora_settings.signaling_urls,
            channel_id=sora_settings.channel_id,
            role="sendonly",
            audio=True,
            video=True,
            metadata=sora_settings.metadata,
            ca_cert=ca_cert,
            insecure=False,
        ),
    ):
        # 起動に失敗する想定。ここには到達しない。
        pytest.fail("自己発行 CA のみで Momo プロセスが起動してはいけない")


def test_sora_turn_tls_relay_protocol_is_tls(sora_settings, free_port):
    """
    Sora + JWT turn_tls_only で TURN-TLS 経路を使い、relayProtocol が tls であることを確認する。

    前提:
    - 信頼ストアは未指定（システム CA）
    - Sora Labo の JWT クレーム turn_tls_only=true で TURN-TLS を強制する

    期待:
    - 接続が確立し、local-candidate (relay) の relayProtocol が tls になる
    """
    metadata = _metadata_with_turn_tls_only(sora_settings)
    print(f"TURN-TLS ケース: channel_id={sora_settings.channel_id}")

    with Momo(
        mode=MomoMode.SORA,
        metrics_port=free_port,
        fake_capture_device=True,
        signaling_urls=sora_settings.signaling_urls,
        channel_id=sora_settings.channel_id,
        role="sendonly",
        audio=True,
        video=True,
        metadata=metadata,
    ) as m:
        assert m.wait_for_connection(timeout=30), "TURN-TLS 接続が確立しなかった"

        # relay candidate が揃うまで待ち、relayProtocol が tls であることを確認する
        data = m.get_metrics(
            wait_stats=[
                {
                    "type": "local-candidate",
                    "candidateType": "relay",
                    "relayProtocol": "tls",
                }
            ],
            wait_stats_timeout=30,
        )

        relay_candidates = [
            stat
            for stat in data.get("stats", [])
            if stat.get("type") == "local-candidate" and stat.get("candidateType") == "relay"
        ]
        assert relay_candidates, "relay の local-candidate が見つからない"
        assert any(stat.get("relayProtocol") == "tls" for stat in relay_candidates), (
            f"relayProtocol が tls の candidate が無い: {relay_candidates}"
        )
