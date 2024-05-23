# Momo の統計情報を取得する

## 概要

Momo には統計情報を HTTP API 経由で JSON 形式で取得することができる MetricsServer が内臓されています。ここでは、MetricsServer の起動方法、レスポンスの形式などについて説明します。

## MetricsServer の起動方法

デフォルトでは MetricsServer は起動しません。起動させたい場合は、`--metrics-port` オプションでポート番号を指定してください。8081 番ポートで起動させたい場合は、次のように指定してください。

```bash
./momo --metrics-port 8081 <mode>
```

MetricsServer はデフォルトでループバック (127.0.0.1 で listen) アドレスからのみアクセス可能です。グローバル (0.0.0.0 で listen) アクセスを許可する場合は `--metrics-allow-external-ip` 引数を指定してください。

## 統計情報 API

### URL

統計情報を取得するには HTTP の GET メソッドを利用して `/metrics` にアクセスしてください。例えば、次のオプションで Momo を起動した場合、

```bash
./momo --metrics-port 8081 test
```

<http://127.0.0.1:8081/metrics> にブラウザ、または `curl` などの HTTP クライアントでアクセスすることで統計情報を取得することができます。

### 統計情報 JSON の仕様

統計情報は JSON 形式で取得することができます。JSON に含まれる内容は次の通りです。

```json
{
  "version": "MomoVersion::GetClientName() の戻り値",
  "environment": "MomoVersion::GetEnvironmentName() の戻り値",
  "libwebrtc": "MomoVersion::GetLibwebrtcName() の戻り値",
  "stats": [`werbrtc::RTCStats`, ...] // Sora モードの pong メッセージに含まれるものと同じ"
}
```

実際のレスポンスの例は次のようになります。

```json
{
  "version": "WebRTC Native Client Momo 2020.11 (39607520)",
  "environment": "[aarch64] Ubuntu 18.04.5 LTS (nvidia-l4t-core 32.4.4-20201016123640)",
  "libwebrtc": "Shiguredo-Build M88.4324@{#3} (88.4324.3.0 b15b2915)",
  "stats": [
    {
      "base64Certificate": "MIIBFjCBvaADAgECAgkApU7zxF5fhbcwCgYIKoZIzj0EAwIwETEPMA0GA1UEAwwGV2ViUlRDMB4XDTIwMTIxODA1Mjc0OVoXDTIxMDExODA1Mjc0OVowETEPMA0GA1UEAwwGV2ViUlRDMFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEDwRrT2pCTn8STtVcaG0Jy78ZGcW2Kl+DjvJyveMA/4avzcHRtmAfB8R0197uxvFPDPE+MVwAp3xJjfxPnbNeajAKBggqhkjOPQQDAgNIADBFAiBPKd9HcryWjjL9mhdnqeHbvdjLa/aDzT9OVNChJ0tN9QIhANayX7S64nedOQPG5COZohnAicvNMzhpuJMgxHXhSByy",
      "fingerprint": "39:BD:A1:C5:40:EB:D4:DA:40:7D:29:E5:DC:70:21:F0:C7:0C:1D:2C:4C:CE:38:C5:11:CB:0F:AD:6D:A0:B1:DF",
      "fingerprintAlgorithm": "sha-256",
      "id": "RTCCertificate_39:BD:A1:C5:40:EB:D4:DA:40:7D:29:E5:DC:70:21:F0:C7:0C:1D:2C:4C:CE:38:C5:11:CB:0F:AD:6D:A0:B1:DF",
      "timestamp": 1608355682366521,
      "type": "certificate"
    },
    {
      "base64Certificate": "MIIBFTCBvaADAgECAgkAijBOQg68QKUwCgYIKoZIzj0EAwIwETEPMA0GA1UEAwwGV2ViUlRDMB4XDTIwMTIxODA1MjczNVoXDTIxMDExODA1MjczNVowETEPMA0GA1UEAwwGV2ViUlRDMFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEK9br1cWzDzvMnP+0d6e/RvPgFdwvMt5pqm2RHHweOCsJJqWthSk7S3l5Yve48cDqED0vQ6JPQXdlaVP+MqZuBTAKBggqhkjOPQQDAgNHADBEAiA83BHLPl1tUJDXykYaCBhegWVfr4rUqwP3JPY/99ZZEQIgY1Iik0NrGENgM+nxLbR1/W4M52khQ6rB5CS6PN63mtM=",
      "fingerprint": "D5:76:46:FA:F4:9A:CD:6F:95:0A:A1:35:4D:FC:D1:62:89:0B:F8:B7:B3:91:EE:A0:35:78:1A:04:B7:1F:75:3E",
      "fingerprintAlgorithm": "sha-256",
      "id": "RTCCertificate_D5:76:46:FA:F4:9A:CD:6F:95:0A:A1:35:4D:FC:D1:62:89:0B:F8:B7:B3:91:EE:A0:35:78:1A:04:B7:1F:75:3E",
      "timestamp": 1608355682366521,
      "type": "certificate"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_102",
      "mimeType": "video/H264",
      "payloadType": 102,
      "sdpFmtpLine": "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42001f",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_107",
      "mimeType": "video/rtx",
      "payloadType": 107,
      "sdpFmtpLine": "apt=125",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_108",
      "mimeType": "video/H264",
      "payloadType": 108,
      "sdpFmtpLine": "level-asymmetry-allowed=1;packetization-mode=0;profile-level-id=42e01f",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_109",
      "mimeType": "video/rtx",
      "payloadType": 109,
      "sdpFmtpLine": "apt=108",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_114",
      "mimeType": "video/red",
      "payloadType": 114,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_115",
      "mimeType": "video/rtx",
      "payloadType": 115,
      "sdpFmtpLine": "apt=114",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_116",
      "mimeType": "video/ulpfec",
      "payloadType": 116,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_120",
      "mimeType": "video/rtx",
      "payloadType": 120,
      "sdpFmtpLine": "apt=127",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_121",
      "mimeType": "video/rtx",
      "payloadType": 121,
      "sdpFmtpLine": "apt=102",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_125",
      "mimeType": "video/H264",
      "payloadType": 125,
      "sdpFmtpLine": "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Inbound_127",
      "mimeType": "video/H264",
      "payloadType": 127,
      "sdpFmtpLine": "level-asymmetry-allowed=1;packetization-mode=0;profile-level-id=42001f",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_102",
      "mimeType": "video/H264",
      "payloadType": 102,
      "sdpFmtpLine": "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42001f",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_107",
      "mimeType": "video/rtx",
      "payloadType": 107,
      "sdpFmtpLine": "apt=125",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_108",
      "mimeType": "video/H264",
      "payloadType": 108,
      "sdpFmtpLine": "level-asymmetry-allowed=1;packetization-mode=0;profile-level-id=42e01f",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_109",
      "mimeType": "video/rtx",
      "payloadType": 109,
      "sdpFmtpLine": "apt=108",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_114",
      "mimeType": "video/red",
      "payloadType": 114,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_115",
      "mimeType": "video/rtx",
      "payloadType": 115,
      "sdpFmtpLine": "apt=114",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_116",
      "mimeType": "video/ulpfec",
      "payloadType": 116,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_118",
      "mimeType": "video/rtx",
      "payloadType": 118,
      "sdpFmtpLine": "apt=123",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_119",
      "mimeType": "video/rtx",
      "payloadType": 119,
      "sdpFmtpLine": "apt=124",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_120",
      "mimeType": "video/rtx",
      "payloadType": 120,
      "sdpFmtpLine": "apt=127",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_121",
      "mimeType": "video/rtx",
      "payloadType": 121,
      "sdpFmtpLine": "apt=102",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_123",
      "mimeType": "video/H264",
      "payloadType": 123,
      "sdpFmtpLine": "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=64001f",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_124",
      "mimeType": "video/H264",
      "payloadType": 124,
      "sdpFmtpLine": "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=4d001f",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_125",
      "mimeType": "video/H264",
      "payloadType": 125,
      "sdpFmtpLine": "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "clockRate": 90000,
      "id": "RTCCodec_0_Outbound_127",
      "mimeType": "video/H264",
      "payloadType": 127,
      "sdpFmtpLine": "level-asymmetry-allowed=1;packetization-mode=0;profile-level-id=42001f",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 8000,
      "id": "RTCCodec_1_Inbound_0",
      "mimeType": "audio/PCMU",
      "payloadType": 0,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 16000,
      "id": "RTCCodec_1_Inbound_103",
      "mimeType": "audio/ISAC",
      "payloadType": 103,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 32000,
      "id": "RTCCodec_1_Inbound_104",
      "mimeType": "audio/ISAC",
      "payloadType": 104,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 16000,
      "id": "RTCCodec_1_Inbound_105",
      "mimeType": "audio/CN",
      "payloadType": 105,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 32000,
      "id": "RTCCodec_1_Inbound_106",
      "mimeType": "audio/CN",
      "payloadType": 106,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 48000,
      "id": "RTCCodec_1_Inbound_110",
      "mimeType": "audio/telephone-event",
      "payloadType": 110,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 2,
      "clockRate": 48000,
      "id": "RTCCodec_1_Inbound_111",
      "mimeType": "audio/opus",
      "payloadType": 111,
      "sdpFmtpLine": "minptime=10;useinbandfec=1",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 32000,
      "id": "RTCCodec_1_Inbound_112",
      "mimeType": "audio/telephone-event",
      "payloadType": 112,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 16000,
      "id": "RTCCodec_1_Inbound_113",
      "mimeType": "audio/telephone-event",
      "payloadType": 113,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 8000,
      "id": "RTCCodec_1_Inbound_126",
      "mimeType": "audio/telephone-event",
      "payloadType": 126,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 8000,
      "id": "RTCCodec_1_Inbound_13",
      "mimeType": "audio/CN",
      "payloadType": 13,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 8000,
      "id": "RTCCodec_1_Inbound_8",
      "mimeType": "audio/PCMA",
      "payloadType": 8,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 8000,
      "id": "RTCCodec_1_Inbound_9",
      "mimeType": "audio/G722",
      "payloadType": 9,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 8000,
      "id": "RTCCodec_1_Outbound_0",
      "mimeType": "audio/PCMU",
      "payloadType": 0,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 16000,
      "id": "RTCCodec_1_Outbound_103",
      "mimeType": "audio/ISAC",
      "payloadType": 103,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 32000,
      "id": "RTCCodec_1_Outbound_104",
      "mimeType": "audio/ISAC",
      "payloadType": 104,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 16000,
      "id": "RTCCodec_1_Outbound_105",
      "mimeType": "audio/CN",
      "payloadType": 105,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 32000,
      "id": "RTCCodec_1_Outbound_106",
      "mimeType": "audio/CN",
      "payloadType": 106,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 48000,
      "id": "RTCCodec_1_Outbound_110",
      "mimeType": "audio/telephone-event",
      "payloadType": 110,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 2,
      "clockRate": 48000,
      "id": "RTCCodec_1_Outbound_111",
      "mimeType": "audio/opus",
      "payloadType": 111,
      "sdpFmtpLine": "minptime=10;useinbandfec=1",
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 32000,
      "id": "RTCCodec_1_Outbound_112",
      "mimeType": "audio/telephone-event",
      "payloadType": 112,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 16000,
      "id": "RTCCodec_1_Outbound_113",
      "mimeType": "audio/telephone-event",
      "payloadType": 113,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 8000,
      "id": "RTCCodec_1_Outbound_126",
      "mimeType": "audio/telephone-event",
      "payloadType": 126,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 8000,
      "id": "RTCCodec_1_Outbound_13",
      "mimeType": "audio/CN",
      "payloadType": 13,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 8000,
      "id": "RTCCodec_1_Outbound_8",
      "mimeType": "audio/PCMA",
      "payloadType": 8,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "channels": 1,
      "clockRate": 8000,
      "id": "RTCCodec_1_Outbound_9",
      "mimeType": "audio/G722",
      "payloadType": 9,
      "timestamp": 1608355682366521,
      "type": "codec"
    },
    {
      "bytesReceived": 0,
      "bytesSent": 0,
      "dataChannelIdentifier": 1,
      "id": "RTCDataChannel_1",
      "label": "serial",
      "messagesReceived": 0,
      "messagesSent": 0,
      "protocol": "",
      "state": "open",
      "timestamp": 1608355682366521,
      "type": "data-channel"
    },
    {
      "availableOutgoingBitrate": 1938532,
      "bytesReceived": 14101,
      "bytesSent": 2711100,
      "consentRequestsSent": 8,
      "currentRoundTripTime": 0.011,
      "id": "RTCIceCandidatePair_6AhCrsVV_kY8TpKYP",
      "localCandidateId": "RTCIceCandidate_6AhCrsVV",
      "nominated": true,
      "priority": 9079290933588803000,
      "remoteCandidateId": "RTCIceCandidate_kY8TpKYP",
      "requestsReceived": 9,
      "requestsSent": 1,
      "responsesReceived": 9,
      "responsesSent": 9,
      "state": "succeeded",
      "timestamp": 1608355682366521,
      "totalRoundTripTime": 0.078,
      "transportId": "RTCTransport_0_1",
      "type": "candidate-pair",
      "writable": true
    },
    {
      "candidateType": "host",
      "deleted": false,
      "id": "RTCIceCandidate_6AhCrsVV",
      "ip": "192.168.1.10",
      "isRemote": false,
      "networkType": "wifi",
      "port": 50153,
      "priority": 2122194687,
      "protocol": "udp",
      "timestamp": 1608355682366521,
      "transportId": "RTCTransport_0_1",
      "type": "local-candidate"
    },
    {
      "candidateType": "host",
      "deleted": false,
      "id": "RTCIceCandidate_kY8TpKYP",
      "ip": "",
      "isRemote": true,
      "port": 57197,
      "priority": 2113937151,
      "protocol": "udp",
      "timestamp": 1608355682366521,
      "transportId": "RTCTransport_0_1",
      "type": "remote-candidate"
    },
    {
      "detached": false,
      "ended": false,
      "frameHeight": 720,
      "frameWidth": 1280,
      "framesSent": 389,
      "hugeFramesSent": 0,
      "id": "RTCMediaStreamTrack_sender_1",
      "kind": "video",
      "mediaSourceId": "RTCVideoSource_1",
      "remoteSource": false,
      "timestamp": 1608355682366521,
      "trackIdentifier": "ykcVDzTtrP0Ntf/RmM6vdVAQmFHvzxlG",
      "type": "track"
    },
    {
      "id": "RTCMediaStream_AXHD4LCG95BuT857oC27rfxW7/U70fXL",
      "streamIdentifier": "AXHD4LCG95BuT857oC27rfxW7/U70fXL",
      "timestamp": 1608355682366521,
      "trackIds": [
        "RTCMediaStreamTrack_sender_1"
      ],
      "type": "stream"
    },
    {
      "bytesSent": 2615235,
      "codecId": "RTCCodec_0_Outbound_102",
      "encoderImplementation": "Jetson Video Encoder",
      "firCount": 0,
      "frameHeight": 720,
      "frameWidth": 1280,
      "framesEncoded": 389,
      "framesPerSecond": 30,
      "framesSent": 389,
      "headerBytesSent": 63856,
      "hugeFramesSent": 0,
      "id": "RTCOutboundRTPVideoStream_3283778679",
      "isRemote": false,
      "keyFramesEncoded": 6,
      "kind": "video",
      "mediaSourceId": "RTCVideoSource_1",
      "mediaType": "video",
      "nackCount": 0,
      "packetsSent": 2451,
      "pliCount": 0,
      "qpSum": 7296,
      "qualityLimitationReason": "none",
      "qualityLimitationResolutionChanges": 0,
      "remoteId": "RTCRemoteInboundRtpVideoStream_3283778679",
      "retransmittedBytesSent": 0,
      "retransmittedPacketsSent": 0,
      "ssrc": 3283778679,
      "timestamp": 1608355682366521,
      "totalEncodeTime": 6.576,
      "totalEncodedBytesTarget": 0,
      "totalPacketSendDelay": 159.614,
      "trackId": "RTCMediaStreamTrack_sender_1",
      "transportId": "RTCTransport_0_1",
      "type": "outbound-rtp"
    },
    {
      "dataChannelsClosed": 0,
      "dataChannelsOpened": 1,
      "id": "RTCPeerConnection",
      "timestamp": 1608355682366521,
      "type": "peer-connection"
    },
    {
      "codecId": "RTCCodec_0_Outbound_102",
      "id": "RTCRemoteInboundRtpVideoStream_3283778679",
      "jitter": 0.0317,
      "kind": "video",
      "localId": "RTCOutboundRTPVideoStream_3283778679",
      "packetsLost": 0,
      "roundTripTime": 0.04,
      "ssrc": 3283778679,
      "timestamp": 1608355681950059,
      "transportId": "RTCTransport_0_1",
      "type": "remote-inbound-rtp"
    },
    {
      "bytesReceived": 14101,
      "bytesSent": 2711100,
      "dtlsCipher": "TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256",
      "dtlsState": "connected",
      "id": "RTCTransport_0_1",
      "localCertificateId": "RTCCertificate_39:BD:A1:C5:40:EB:D4:DA:40:7D:29:E5:DC:70:21:F0:C7:0C:1D:2C:4C:CE:38:C5:11:CB:0F:AD:6D:A0:B1:DF",
      "packetsReceived": 260,
      "packetsSent": 2516,
      "remoteCertificateId": "RTCCertificate_D5:76:46:FA:F4:9A:CD:6F:95:0A:A1:35:4D:FC:D1:62:89:0B:F8:B7:B3:91:EE:A0:35:78:1A:04:B7:1F:75:3E",
      "selectedCandidatePairChanges": 1,
      "selectedCandidatePairId": "RTCIceCandidatePair_6AhCrsVV_kY8TpKYP",
      "srtpCipher": "AES_CM_128_HMAC_SHA1_80",
      "timestamp": 1608355682366521,
      "tlsVersion": "FEFD",
      "type": "transport"
    },
    {
      "framesPerSecond": 30,
      "height": 720,
      "id": "RTCVideoSource_1",
      "kind": "video",
      "timestamp": 1608355682366521,
      "trackIdentifier": "ykcVDzTtrP0Ntf/RmM6vdVAQmFHvzxlG",
      "type": "media-source",
      "width": 1280
    }
  ]
}
```

## WebRTC 統計情報の仕様

統計情報 API のレスポンスに含まれる `stats` フィールドの詳細は W3C の標準仕様 [Identifiers for WebRTC's Statistics API](https://www.w3.org/TR/webrtc-stats/) を参考にしてください。

ただし、 2020 年 12 月時点で、この標準仕様はドラフトバージョンであり、また、Momo が利用している libwebrtc バージョンによっては実装されていないものもありますので、注意してください。

## 応用例

- [自宅の Jetson で動いている WebRTC Native Client Momo を外出先でいい感じに監視する方法](https://zenn.dev/hakobera/articles/c0553faa1223324d6aff)
