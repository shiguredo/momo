"""Ayame モードの E2E テスト"""

import uuid

import pytest

from momo import Momo, MomoMode

AYAME_SIGNALING_URL = "wss://ayame-labo.shiguredo.app/signaling"


def test_ayame_mode_basic(free_port, port_allocator):
    """Ayame モードで momo を起動できることを確認"""
    room_id = str(uuid.uuid4())
    
    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        metrics_port=free_port,
        fake_capture_device=True,
        resolution="QVGA",
        framerate=30,
        log_level="info",
    ) as m:
        data = m.get_metrics()
        assert "version" in data


def test_ayame_mode_with_client_id(free_port, port_allocator):
    """Ayame モードで client_id を指定して起動できることを確認"""
    room_id = str(uuid.uuid4())
    client_id = str(uuid.uuid4())
    
    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        client_id=client_id,
        metrics_port=free_port,
        fake_capture_device=True,
        resolution="QVGA",
        framerate=15,
    ) as m:
        data = m.get_metrics()
        assert "version" in data  # メトリクスが取得できることを確認



def test_ayame_mode_with_video_settings(free_port, port_allocator):
    """Ayame モードでビデオ設定をカスタマイズして起動できることを確認"""
    room_id = str(uuid.uuid4())
    
    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        metrics_port=free_port,
        fake_capture_device=True,
        resolution="VGA",
        framerate=60,
        vp8_encoder="software",
        log_level="verbose",
    ) as m:
        data = m.get_metrics()
        assert "version" in data  # メトリクスが取得できることを確認


@pytest.mark.skip(reason="コーデック強制機能が未実装のため")
@pytest.mark.parametrize("codec", ["VP8", "VP9", "AV1"])
def test_ayame_mode_with_codec(port_allocator, codec):
    """Ayame モードで各種コーデックを使用した通信を確認"""
    room_id = str(uuid.uuid4())
    
    # コーデックごとのエンコーダー/デコーダー設定
    codec_settings = {}
    if codec == "VP8":
        codec_settings = {
            "vp8_encoder": "software",
            "vp8_decoder": "software",
        }
    elif codec == "VP9":
        codec_settings = {
            "vp9_encoder": "software",
            "vp9_decoder": "software",
        }
    elif codec == "AV1":
        codec_settings = {
            "av1_encoder": "software",
            "av1_decoder": "software",
        }
    
    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        client_id=str(uuid.uuid4()),
        metrics_port=next(port_allocator),
        fake_capture_device=True,
        resolution="QVGA",
        **codec_settings,
    ) as peer1:
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url=AYAME_SIGNALING_URL,
            room_id=room_id,
            client_id=str(uuid.uuid4()),
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            resolution="QVGA",
            **codec_settings,
        ) as peer2:
            # 接続が確立されるまで待機（片方が繋がれば両方繋がるはず）
            assert peer1.wait_for_connection(timeout=10), \
                f"Failed to establish peer connection for {codec} codec within timeout"
            
            peer1_data = peer1.get_metrics()
            peer2_data = peer2.get_metrics()
            
            # peer1 の outbound-rtp でコーデックを確認
            peer1_outbound = next(
                (stat for stat in peer1_data["stats"] 
                 if stat.get("type") == "outbound-rtp" and stat.get("kind") == "video"),
                None
            )
            assert peer1_outbound is not None, "Could not find peer1 outbound-rtp video stream"
            
            codec_id = peer1_outbound.get("codecId")
            assert codec_id is not None, "No codecId found in peer1 outbound-rtp stats"
            
            # codecId から codec 統計を探す
            peer1_codec = next(
                (stat for stat in peer1_data["stats"] 
                 if stat.get("id") == codec_id and stat.get("type") == "codec"),
                None
            )
            assert peer1_codec is not None, f"Could not find codec stats for codecId: {codec_id}"
            
            mime_type = peer1_codec.get("mimeType", "")
            assert codec in mime_type.upper(), f"Expected {codec} codec but got: {mime_type}"
            print(f"Peer1 codec for {codec}: {mime_type}")
            
            # peer2 の outbound-rtp でもコーデックを確認（双方向通信なので）
            peer2_outbound = next(
                (stat for stat in peer2_data["stats"] 
                 if stat.get("type") == "outbound-rtp" and stat.get("kind") == "video"),
                None
            )
            assert peer2_outbound is not None, "Could not find peer2 outbound-rtp video stream"
            
            codec_id = peer2_outbound.get("codecId")
            assert codec_id is not None, "No codecId found in peer2 outbound-rtp stats"
            
            # codecId から codec 統計を探す
            peer2_codec = next(
                (stat for stat in peer2_data["stats"] 
                 if stat.get("id") == codec_id and stat.get("type") == "codec"),
                None
            )
            assert peer2_codec is not None, f"Could not find codec stats for codecId: {codec_id}"
            
            mime_type = peer2_codec.get("mimeType", "")
            assert codec in mime_type.upper(), f"Expected {codec} codec but got: {mime_type}"
            print(f"Peer2 codec for {codec}: {mime_type}")


def test_ayame_mode_with_audio_settings(free_port, port_allocator):
    """Ayame モードでオーディオ設定をカスタマイズして起動できることを確認"""
    room_id = str(uuid.uuid4())
    
    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        metrics_port=free_port,
        fake_capture_device=True,
        disable_echo_cancellation=True,
        disable_auto_gain_control=True,
        disable_noise_suppression=True,
    ) as m:
        data = m.get_metrics()
        assert "version" in data  # メトリクスが取得できることを確認


def test_ayame_mode_peer_connection(port_allocator):
    """Ayame モードで2つのピアが双方向通信できることを確認"""
    room_id = str(uuid.uuid4())
    
    with Momo(
        mode=MomoMode.AYAME,
        ayame_signaling_url=AYAME_SIGNALING_URL,
        room_id=room_id,
        client_id=str(uuid.uuid4()),
        metrics_port=next(port_allocator),
        fake_capture_device=True,
        resolution="QVGA",
    ) as peer1:
        with Momo(
            mode=MomoMode.AYAME,
            ayame_signaling_url=AYAME_SIGNALING_URL,
            room_id=room_id,
            client_id=str(uuid.uuid4()),
            metrics_port=next(port_allocator),
            fake_capture_device=True,
            resolution="QVGA",
        ) as peer2:
            # 接続が確立されるまで待機（片方が繋がれば両方繋がるはず）
            assert peer1.wait_for_connection(timeout=10), \
                "Failed to establish peer connection within timeout"
            
            peer1_data = peer1.get_metrics()
            peer2_data = peer2.get_metrics()
            
            # stats が存在することを確認
            assert "stats" in peer1_data
            assert "stats" in peer2_data
            assert len(peer1_data["stats"]) > 0
            assert len(peer2_data["stats"]) > 0
            
            # peer1 の送受信を確認（送信と受信の両方があるはず）
            peer1_video_out = next(
                (stat for stat in peer1_data["stats"] 
                 if stat.get("type") == "outbound-rtp" and stat.get("kind") == "video"),
                None
            )
            peer1_video_in = next(
                (stat for stat in peer1_data["stats"] 
                 if stat.get("type") == "inbound-rtp" and stat.get("kind") == "video"),
                None
            )
            assert peer1_video_out is not None, "Peer1 should have video outbound-rtp"
            assert peer1_video_in is not None, "Peer1 should have video inbound-rtp"
            assert peer1_video_out.get("packetsSent", 0) > 0
            assert peer1_video_in.get("packetsReceived", 0) > 0
            
            # peer2 の送受信を確認（送信と受信の両方があるはず）
            peer2_video_out = next(
                (stat for stat in peer2_data["stats"] 
                 if stat.get("type") == "outbound-rtp" and stat.get("kind") == "video"),
                None
            )
            peer2_video_in = next(
                (stat for stat in peer2_data["stats"] 
                 if stat.get("type") == "inbound-rtp" and stat.get("kind") == "video"),
                None
            )
            assert peer2_video_out is not None, "Peer2 should have video outbound-rtp"
            assert peer2_video_in is not None, "Peer2 should have video inbound-rtp"
            assert peer2_video_out.get("packetsSent", 0) > 0
            assert peer2_video_in.get("packetsReceived", 0) > 0
            
            print(f"Peer1 video - sent: {peer1_video_out.get('packetsSent')}, received: {peer1_video_in.get('packetsReceived')}")
            print(f"Peer2 video - sent: {peer2_video_out.get('packetsSent')}, received: {peer2_video_in.get('packetsReceived')}")