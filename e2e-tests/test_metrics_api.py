"""メトリクス API の E2E テスト"""

import platform
import re
import socket
import uuid

import httpx
import pytest

from momo import Momo, MomoMode, find_all_stats, find_stats

AYAME_SIGNALING_URL = "wss://ayame-labo.shiguredo.app/signaling"


def _get_non_loopback_ipv4() -> str | None:
    """ホストの非ループバック IPv4 アドレスを取得

    取得できない場合や、ループバックしか取得できない場合は None を返す。
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # 実際にパケットは送信されない（UDP の connect は経路選択のみ行う）
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()
    if ip.startswith("127.") or ip == "0.0.0.0":
        return None
    return ip


def test_metrics_response_structure(free_port, port_allocator):
    """メトリクスレスポンスの JSON 構造を確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
    ) as m:
        # get_metrics() は内部で raise_for_status() を呼ぶので 200 でない場合は例外が発生する
        data = m.get_metrics()
        assert data is not None
        assert isinstance(data, dict)

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


def test_invalid_endpoint_returns_404(free_port, port_allocator):
    """存在しないエンドポイントが 404 を返すことを確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
    ) as m:
        assert m._http_client is not None
        response = m._http_client.get(f"http://localhost:{m.metrics_port}/invalid")
        assert response.status_code == 404


def test_post_method_returns_error(free_port, port_allocator):
    """POST メソッドがエラーを返すことを確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
    ) as m:
        assert m._http_client is not None
        response = m._http_client.post(f"http://localhost:{m.metrics_port}/metrics")
        assert response.status_code == 400  # Bad Request


def test_get_metrics_method(free_port, port_allocator):
    """get_metrics メソッドが正しく動作することを確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
    ) as m:
        # get_metrics メソッドを使用
        metrics = m.get_metrics()

        assert isinstance(metrics, dict)
        assert "version" in metrics
        assert "libwebrtc" in metrics


def test_metrics_version_format(free_port, port_allocator):
    """version フィールドが C++ 版 momo と互換のフォーマットであることを確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
    ) as m:
        data = m.get_metrics()
        version = data["version"]

        # "WebRTC Native Client Momo {version}" 形式
        assert version.startswith("WebRTC Native Client Momo "), (
            f"Unexpected version format: {version}"
        )
        # バージョン番号部分が存在する
        suffix = version[len("WebRTC Native Client Momo ") :]
        assert len(suffix) > 0, f"Version suffix is empty: {version}"


def test_metrics_libwebrtc_format(free_port, port_allocator):
    """libwebrtc フィールドが C++ 版 momo と互換のフォーマットであることを確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
    ) as m:
        data = m.get_metrics()
        libwebrtc = data["libwebrtc"]

        # "webrtc-rs {version}" 形式
        assert libwebrtc.startswith("webrtc-rs "), f"Unexpected libwebrtc format: {libwebrtc}"
        # バージョン部分が存在する
        suffix = libwebrtc[len("webrtc-rs ") :]
        assert len(suffix) > 0, f"libwebrtc version suffix is empty: {libwebrtc}"


def test_metrics_environment_format(free_port, port_allocator):
    """environment フィールドが "[ARCH] OS_DETAIL" 形式であることを確認"""
    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
    ) as m:
        data = m.get_metrics()
        environment = data["environment"]

        # "[ARCH] OS_DETAIL" 形式の検証
        match = re.match(r"^\[([^\]]+)\] (.+)$", environment)
        assert match is not None, f"Unexpected environment format: {environment}"

        arch, os_detail = match.group(1), match.group(2)
        assert len(arch) > 0, f"ARCH is empty: {environment}"
        assert len(os_detail) > 0, f"OS_DETAIL is empty: {environment}"

        # ARCH が現在のプロセスの値と一致する
        # std::env::consts::ARCH と platform.machine() の対応関係を考慮
        machine = platform.machine().lower()
        # macOS: arm64, x86_64 / Linux: aarch64, x86_64 / Windows: AMD64 → x86_64
        expected_archs: set[str]
        match machine:
            case "x86_64" | "amd64":
                expected_archs = {"x86_64"}
            case "arm64" | "aarch64":
                expected_archs = {"aarch64"}
            case _:
                expected_archs = {machine}
        assert arch in expected_archs, f"Unexpected ARCH: {arch}, expected one of: {expected_archs}"

        # OS_DETAIL が OS に応じた形式であることを確認
        system = platform.system()
        if system == "Darwin":
            # "macOS X.Y" 形式
            assert os_detail.startswith("macOS"), f"Unexpected macOS detail: {os_detail}"
        elif system == "Linux":
            # /etc/os-release の PRETTY_NAME か "Linux" が返る
            assert len(os_detail) > 0


def test_metrics_allow_external_ip_default_localhost_only(free_port, port_allocator):
    """--metrics-allow-external-ip 未指定時は 127.0.0.1 でのみバインドされることを確認"""
    external_ip = _get_non_loopback_ipv4()
    if external_ip is None:
        pytest.skip("Non-loopback IPv4 address is not available")

    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
        metrics_allow_external_ip=False,
    ) as m:
        # 127.0.0.1 で接続できる
        with httpx.Client(timeout=2.0) as client:
            response = client.get(f"http://127.0.0.1:{m.metrics_port}/metrics")
            assert response.status_code == 200

            # 非ループバック IP では接続できない（バインドされていないので拒否される）
            with pytest.raises((httpx.ConnectError, httpx.ConnectTimeout, httpx.ReadTimeout)):
                client.get(f"http://{external_ip}:{m.metrics_port}/metrics")


def test_metrics_allow_external_ip_enabled(free_port, port_allocator):
    """--metrics-allow-external-ip 指定時は非ループバック IP からも接続できることを確認"""
    external_ip = _get_non_loopback_ipv4()
    if external_ip is None:
        pytest.skip("Non-loopback IPv4 address is not available")

    with Momo(
        mode=MomoMode.P2P,
        metrics_port=free_port,
        port=next(port_allocator),
        fake_capture_device=True,
        metrics_allow_external_ip=True,
    ) as m:
        # 127.0.0.1 でも非ループバック IP でも接続できる
        with httpx.Client(timeout=5.0) as client:
            response = client.get(f"http://127.0.0.1:{m.metrics_port}/metrics")
            assert response.status_code == 200

            response = client.get(f"http://{external_ip}:{m.metrics_port}/metrics")
            assert response.status_code == 200


def test_metrics_stats_structure_after_connection(port_allocator):
    """WebRTC 接続後の stats レスポンス構造が WebRTC 仕様と互換であることを確認"""
    room_id = str(uuid.uuid4())

    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        client_id=str(uuid.uuid4()),
        metrics_port=next(port_allocator),
        fake_capture_device=True,
        resolution="QVGA",
    ) as m1:
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url=AYAME_SIGNALING_URL,
            room_id=room_id,
            client_id=str(uuid.uuid4()),
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            resolution="QVGA",
        ) as m2:
            assert m1.wait_for_connection(timeout=10), "M1 failed to connect"
            assert m2.wait_for_connection(timeout=10), "M2 failed to connect"

            # outbound-rtp と inbound-rtp の両方が届くまで待つ
            data = m1.get_metrics(
                wait_stats=[
                    {"type": "outbound-rtp", "kind": "video"},
                    {"type": "inbound-rtp", "kind": "video"},
                ],
                wait_stats_timeout=10,
            )

            # 全体構造の確認
            assert isinstance(data["stats"], list)
            assert len(data["stats"]) > 0

            # 全 stats エントリは id, type フィールドを持つ
            for stat in data["stats"]:
                assert "id" in stat, f"stat missing id: {stat}"
                assert "type" in stat, f"stat missing type: {stat}"
                assert isinstance(stat["id"], str)
                assert isinstance(stat["type"], str)

            # transport の構造検証
            transport = find_stats(data, type="transport")
            assert transport is not None, "transport stats not found"
            assert transport["dtlsState"] == "connected"
            assert transport["iceState"] == "connected"
            assert isinstance(transport["bytesSent"], int)
            assert isinstance(transport["bytesReceived"], int)
            assert "selectedCandidatePairId" in transport

            # peer-connection の構造検証
            peer_connection = find_stats(data, type="peer-connection")
            assert peer_connection is not None, "peer-connection stats not found"

            # candidate-pair の構造検証（selectedCandidatePairId が指す pair が存在する）
            candidate_pair = find_stats(
                data, type="candidate-pair", id=transport["selectedCandidatePairId"]
            )
            assert candidate_pair is not None, (
                f"selected candidate-pair not found: {transport['selectedCandidatePairId']}"
            )
            assert "state" in candidate_pair
            assert "localCandidateId" in candidate_pair
            assert "remoteCandidateId" in candidate_pair

            # local-candidate / remote-candidate の構造検証
            local_candidate = find_stats(
                data, type="local-candidate", id=candidate_pair["localCandidateId"]
            )
            assert local_candidate is not None, "local-candidate not found"
            assert "candidateType" in local_candidate
            assert "protocol" in local_candidate
            assert "port" in local_candidate

            remote_candidate = find_stats(
                data, type="remote-candidate", id=candidate_pair["remoteCandidateId"]
            )
            assert remote_candidate is not None, "remote-candidate not found"
            assert "candidateType" in remote_candidate
            assert "protocol" in remote_candidate
            assert "port" in remote_candidate

            # outbound-rtp video の構造検証
            outbound_video = find_stats(data, type="outbound-rtp", kind="video")
            assert outbound_video is not None, "outbound-rtp video not found"
            assert isinstance(outbound_video["ssrc"], int)
            assert outbound_video["packetsSent"] > 0
            assert outbound_video["bytesSent"] > 0
            assert "codecId" in outbound_video

            # outbound-rtp が参照する codec 情報が存在する
            outbound_codec = find_stats(data, id=outbound_video["codecId"], type="codec")
            assert outbound_codec is not None, (
                f"codec for outbound-rtp not found: codecId={outbound_video['codecId']}"
            )
            assert outbound_codec["mimeType"].startswith("video/")
            assert isinstance(outbound_codec["payloadType"], int)

            # inbound-rtp video の構造検証
            inbound_video = find_stats(data, type="inbound-rtp", kind="video")
            assert inbound_video is not None, "inbound-rtp video not found"
            assert isinstance(inbound_video["ssrc"], int)
            assert inbound_video["packetsReceived"] > 0
            assert inbound_video["bytesReceived"] > 0
            assert "codecId" in inbound_video

            # inbound-rtp が参照する codec 情報が存在する
            inbound_codec = find_stats(data, id=inbound_video["codecId"], type="codec")
            assert inbound_codec is not None, (
                f"codec for inbound-rtp not found: codecId={inbound_video['codecId']}"
            )
            assert inbound_codec["mimeType"].startswith("video/")

            # codec エントリの一覧が空でない
            codecs = find_all_stats(data, type="codec")
            assert len(codecs) > 0, "no codec stats found"
            for codec in codecs:
                assert "mimeType" in codec
                assert isinstance(codec["payloadType"], int)
