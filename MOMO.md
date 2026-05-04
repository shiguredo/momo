# momo と momo-rs の機能比較・TODO

momo-rs は momo の完全上位互換を目指す。

機能を実装した場合は本ファイルの momo-rs 列の状態を「実装済み」に更新すること。
新しい機能差分が見つかった場合は本ファイルに行を追加すること。

## 凡例

状態:

- 実装済み: 機能が動作する状態
- CLI のみ: CLI オプションは定義済みだが、パース後 `_` 付き変数に束縛されるだけで実処理に渡されていない
- 未実装: CLI オプション自体が存在しない

TODO:

- [ ] 未着手
- [x] 完了

## モード

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| P2P モード | 実装済み | 実装済み | - | HTTP サーバー + WebSocket シグナリング |
| Ayame モード | 実装済み | 実装済み | - | register/accept/reject/offer/answer/candidate |
| Sora モード | 実装済み | 実装済み | #0001 | sora_sdk を使用。SoraServer (`--port` / `--auto`) は未実装 |

### TODO

- [x] Sora モード基本実装 (#0001)
  - [x] WebSocket シグナリング (sora_sdk 内部)
  - [x] DataChannel シグナリング (sora_sdk 内部)
  - [x] マルチストリーム (sora_sdk 内部)
  - [x] ロール制御 (sendonly / recvonly / sendrecv)
  - [x] メタデータ送信 (JSON)
  - [x] サイマルキャスト
  - [x] スポットライト (有効化のみ、配信数制御は sora_sdk API 不足)
  - [x] ビットレート制御 (映像 0-30000 / 音声 0-510)
  - [x] WebSocket 切断時の動作制御 (ignore-disconnect-websocket)
  - [x] メトリクス stats プロバイダー登録
- [ ] Sora モード自動再接続 (#0001)
  - sora_sdk 内部の対応要確認
- [ ] SoraServer (`--port` / `--auto`) の実装 (#0001)
- [ ] Ayame モード自動再接続 (#0002)
  - 現状: momo は watchdog 機構付き自動再接続あり、momo-rs は未実装

### momo の P2P モード実装詳細

- HTTP 静的ファイルサーバー (`--document-root` でルートディレクトリ指定)
- WebSocket シグナリング (`/ws` エンドポイント)
- PeerConnection の作成・管理・破棄
- 映像・音声トラックの送受信
- ICE candidate の交換
- `StatsCollector` 実装でメトリクス API 対応

### momo-rs の P2P モード実装詳細

- HTTP 静的ファイルサーバー: shiguredo_http11 + tokio TcpListener
- WebSocket シグナリング: shiguredo_websocket_connection
- PeerConnection 管理: shiguredo_webrtc
- `--port` (デフォルト 8080)、`--document-root` (デフォルト `html`)
- メトリクス stats プロバイダー登録済み (`src/p2p/websocket.rs` で `peer.pc.get_stats()` を呼び出し)

### momo の Ayame モード実装詳細

- Ayame シグナリングサーバーへの WebSocket 接続
- register / accept / reject メッセージ処理
- offer / answer / candidate の交換
- direction: sendrecv / sendonly / recvonly
- コーデック指定: video-codec-type (VP8/VP9/AV1/H264/H265)、audio-codec-type (OPUS/PCMU/PCMA)
- watchdog 機構付き自動再接続
- `StatsCollector` 実装でメトリクス API 対応

### momo-rs の Ayame モード実装詳細

- Ayame シグナリングサーバーへの WebSocket 接続: shiguredo_websocket_connection
- register / accept / reject / offer / answer / candidate メッセージ処理
- direction: sendrecv / sendonly / recvonly
- コーデック指定: `--video-codec-type` (VP8/VP9/AV1/H264/H265)、`--audio-codec-type` (OPUS/PCMU/PCMA) バリデーション付き
- メトリクス stats プロバイダー登録済み (`src/ayame/mod.rs` で `peer.pc.get_stats()` を呼び出し)
- 自動再接続は未実装

### momo の Sora モード実装詳細

- WebSocket シグナリング (connect / offer / answer / re-offer / re-answer)
- DataChannel シグナリング (WebSocket からの切り替え、タイムアウト設定)
- マルチストリーム
- ロール制御 (sendonly / recvonly / sendrecv)
- メタデータ送信 (JSON)
- サイマルキャスト
- スポットライト (配信数制御)
- 自動再接続
- ビットレート制御 (映像 0-30000 / 音声 0-510)
- WebSocket 切断時の動作制御 (ignore-disconnect-websocket)

### momo-rs の Sora モード実装詳細

- sora_sdk を使用した SoraClient の接続・実行
- 実装済み CLI オプション:
  - `--signaling-urls` (カンマ区切り複数 URL 対応), `--channel-id`, `--video`, `--audio`
  - `--video-codec-type`, `--audio-codec-type`, `--video-bit-rate`, `--audio-bit-rate`
  - `--role` (sendonly/recvonly/sendrecv), `--spotlight`, `--simulcast`
  - `--data-channel-signaling`, `--ignore-disconnect-websocket`, `--disconnect-wait-timeout`
  - `--metadata` (JSON), `--insecure`
- パース済みだが未使用:
  - `--auto`, `--port` (SoraServer 機能が未実装)
- 対応しない:
  - `--spotlight-number` (Sora 側で非推奨・廃止済み)
- 未実装:
  - `--data-channel-signaling-timeout` (sora_sdk に API なし)

## メトリクス API

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| `GET /metrics` エンドポイント | 実装済み | 実装済み | - | JSON レスポンス |
| `--metrics-port` | 実装済み | 実装済み | - | デフォルト -1 (無効) |
| `--metrics-allow-external-ip` | 実装済み (localhost → 全 IP 切り替え) | 実装済み | #0003 | 未指定時 `127.0.0.1`、指定時 `0.0.0.0` |
| レスポンス: `version` | 実装済み | 実装済み | - | |
| レスポンス: `environment` | 実装済み (OS 名/バージョン/アーキテクチャ詳細) | 実装済み | #0005 | `[ARCH] OS_DETAIL` 形式 |
| レスポンス: `libwebrtc` | 実装済み (WEBRTC_READABLE_VERSION) | 実装済み | #0004 | `Shiguredo-Build {version}` |
| レスポンス: `stats` | 実装済み (RTCStatsReport) | 実装済み (RTCStatsReport) | - | 複数 PeerConnection 対応 |
| CORS ヘッダー | 実装済み | 実装済み | - | `Access-Control-Allow-Origin: *` |

### TODO

- [x] `--metrics-allow-external-ip` の実装 (#0003)
- [x] レスポンスに `libwebrtc` フィールドを追加 (#0004)
- [x] レスポンスの `environment` を詳細化 (#0005)

### momo のメトリクス API 実装詳細

- HTTP サーバー: Boost.Beast (HTTP/1.1)
- `StatsCollector` インターフェースで PeerConnection から `RTCStatsReport` を取得
- P2P / Ayame / Sora 全モードが `StatsCollector` を実装
- レスポンス JSON:
  - `version`: `"WebRTC Native Client Momo {VERSION} ({COMMIT_SHORT})"`
  - `libwebrtc`: `"Shiguredo-Build {READABLE_VERSION} ({BUILD_VERSION} {COMMIT_SHORT})"`
  - `environment`: `"[{ARCH}] {OS_DETAIL}"` (Windows: OS バージョン+ビルド番号、macOS: OS 名+バージョン、Linux: /etc/os-release の PRETTY_NAME、Jetson: nvidia-l4t-core バージョン追加)
  - `stats`: `RTCStatsReport::ToJson()` の結果をそのまま JSON 配列として埋め込み
- バインドアドレス: `--metrics-allow-external-ip` なしの場合は `127.0.0.1`、ありの場合は `0.0.0.0`

### momo-rs のメトリクス API 実装詳細

- HTTP サーバー: shiguredo_http11 + tokio TcpListener (HTTP/1.1)
- `MetricsState` が複数の stats プロバイダー (`mpsc::Sender<oneshot::Sender<String>>`) を管理
- PeerConnection 生成時にプロバイダーを `MetricsState::register()` で登録
- P2P (`src/p2p/websocket.rs`)、Ayame (`src/ayame/mod.rs`)、Sora (`src/sora/mod.rs`) で stats JSON を返す
- `collect_stats()` で全プロバイダーに oneshot channel で要求を送信し、各 PeerConnection の `RTCStatsReport` を JSON 配列として結合
- セッション終了済みのプロバイダーは送信失敗時に自動除去
- レスポンス JSON:
  - `version`: `CARGO_PKG_VERSION` の値 (例: `"2026.0.0"`)
  - `environment`: `"[ARCH] OS_DETAIL"` 形式 (例: `"[aarch64] macOS 15.3"`)
  - `libwebrtc`: `"Shiguredo-Build {shiguredo_webrtc::version()}"` 形式
  - `stats`: 各 PeerConnection の `report.to_json()` を結合した JSON 配列
- バインドアドレス: `--metrics-allow-external-ip` 未指定時 `127.0.0.1`、指定時 `0.0.0.0`
- エラーレスポンス: GET 以外は 400、`/metrics` 以外は 404

## 映像キャプチャ

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| V4L2 キャプチャ | 実装済み | 実装済み | - | |
| フェイクキャプチャ | 実装済み | 実装済み | - | `--fake-capture-device` |
| `--video-input-device` | 実装済み | 実装済み | - | MomoConfig に含まれる |
| `--no-video-input-device` | 実装済み | 実装済み | - | MomoConfig に含まれる |
| `--resolution` | 実装済み (QVGA/VGA/HD/FHD/4K/WxH) | 実装済み | #0006 | QVGA/VGA/HD/FHD/4K/WxH 対応 |
| `--framerate` | 実装済み (1-120) | 実装済み | #0006 | |
| `--force-i420` | 実装済み | 実装済み | #0007 | VideoCaptureConfig.pixel_format に I420 を強制 |
| `--force-yuy2` | 実装済み | 実装済み | #0007 | VideoCaptureConfig.pixel_format に YUY2 を強制 |
| `--force-nv12` | 実装済み | 実装済み | #0007 | VideoCaptureConfig.pixel_format に NV12 を強制 |
| `--fixed-resolution` | 実装済み | 実装済み (P2P/Ayame) | #0008 | DegradationPreference::MaintainResolution を強制 |
| `--priority` | 実装済み (BALANCE/FRAMERATE/RESOLUTION) | 実装済み (P2P/Ayame) | #0008 | BALANCE/FRAMERATE/RESOLUTION/DISABLED を DegradationPreference にマッピング。Sora は sora_sdk API 不足 |
| スクリーンキャプチャ | 実装済み (WebRTC DesktopCapturer、マルチディスプレイ対応) | CLI のみ | pending #0019 | `--screen-capture` パース後未使用。shiguredo_screen_capture crate の新規作成が必要 |

### TODO

- [x] `--resolution` の実装 (#0006)
- [x] `--framerate` の実装 (#0006)
- [x] `--force-i420` の実装 (#0007)
- [x] `--force-yuy2` の実装 (#0007)
- [x] `--force-nv12` の実装 (#0007)
- [x] `--fixed-resolution` の実装 (#0008, P2P/Ayame)
- [x] `--priority` の実装 (#0008, P2P/Ayame)
- [ ] `--screen-capture` の実装 (pending #0019)
  - 現状: CLI のみ。WebRTC DesktopCapturer、マルチディスプレイ対応
  - shiguredo_screen_capture crate の新規作成が必要 (macOS: ScreenCaptureKit / Linux Wayland: PipeWire / Linux X11: XShm / Windows: DXGI)

### momo の映像キャプチャ実装詳細

- V4L2VideoCapturer: 標準 V4L2 ビデオキャプチャ (Linux)
- フォーマット対応: I420, YUY2, NV12 + 各フォーマットの強制指定
- 解像度: QVGA (320x240), VGA (640x480), HD (1280x720), FHD (1920x1080), 4K (3840x2160), 任意 WxH
- フレームレート: 1-120fps で任意指定
- `--fixed-resolution`: WebRTC の映像品質劣化時に解像度を維持
- `--priority`: BALANCE / FRAMERATE / RESOLUTION から選択
- スクリーンキャプチャ: WebRTC DesktopCapturer ベース、マルチディスプレイ対応
- フェイクキャプチャ: WebRTC FakeVideoTrackSource

### momo-rs の映像キャプチャ実装詳細

- shiguredo_video_device による V4L2 キャプチャ (Linux)
- 対応ピクセルフォーマット: NV12, YUY2, I420 (自動検出、I420 に変換)
- 解像度: `--resolution` で QVGA/VGA/HD/FHD/4K/WxH 指定 (デフォルト VGA)
- フレームレート: `--framerate` で指定 (デフォルト 30fps)
- フェイクキャプチャ: raden でアニメーションフレーム生成、指定解像度・フレームレート対応
- `--force-i420` / `--force-yuy2` / `--force-nv12`: `VideoCaptureConfig.pixel_format` に強制指定 (排他、同時指定不可)

## 音声

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| `--audio-input-device` | 実装済み | 実装済み | - | MomoConfig に含まれる |
| `--audio-output-device` | 実装済み | CLI のみ | pending #0009 | パース後未使用 |
| `--no-audio-device` | 実装済み | 実装済み | - | MomoConfig に含まれる |
| `--disable-echo-cancellation` | 実装済み (WebRTC BuiltinAudioProcessing) | CLI のみ | pending #0009 | パース後未使用 |
| `--disable-auto-gain-control` | 実装済み (WebRTC BuiltinAudioProcessing) | CLI のみ | pending #0009 | パース後未使用 |
| `--disable-noise-suppression` | 実装済み (WebRTC BuiltinAudioProcessing) | CLI のみ | pending #0009 | パース後未使用 |
| `--disable-highpass-filter` | 実装済み (WebRTC BuiltinAudioProcessing) | CLI のみ | pending #0009 | パース後未使用 |

### TODO

- [ ] 音声処理オプション全般 (pending #0009)
  - shiguredo_webrtc API 不足のため pending
  - 対象: `--audio-output-device`, `--disable-echo-cancellation`, `--disable-auto-gain-control`, `--disable-noise-suppression`, `--disable-highpass-filter`

### momo の音声処理実装詳細

- WebRTC BuiltinAudioProcessing で音声処理パイプラインを構成
- エコーキャンセレーション (AEC): デフォルト有効、`--disable-echo-cancellation` で無効化
- 自動ゲイン制御 (AGC): デフォルト有効、`--disable-auto-gain-control` で無効化
- ノイズ抑制 (NS): デフォルト有効、`--disable-noise-suppression` で無効化
- ハイパスフィルター: デフォルト有効、`--disable-highpass-filter` で無効化
- 入力デバイス指定: インデックス番号またはデバイス名
- 出力デバイス指定: インデックス番号またはデバイス名

### momo-rs の音声実装詳細

- shiguredo_audio_device による音声デバイス列挙・選択
- `--audio-input-device`: インデックス番号またはデバイス名で指定 (MomoConfig に含まれる)
- `--no-audio-device`: 音声を無効化 (MomoConfig に含まれる)
- 音声処理オプション (AEC/AGC/NS/ハイパスフィルター) は CLI でパースされるが実処理に渡されていない

## コーデック

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| VP8 (Software) | 実装済み | 実装済み | - | |
| VP9 (Software) | 実装済み | 実装済み | - | |
| AV1 (Software) | 実装済み | 実装済み | - | |
| H.264 (Software) | 実装済み | 実装済み | - | |
| H.265 | 実装済み | 実装済み | - | |
| `--vp8-encoder` / `--vp8-decoder` | 実装済み (default/software/jetson/nvidia/vpl/videotoolbox/v4l2) | CLI のみ | pending #0010 | パース後未使用 |
| `--vp9-encoder` / `--vp9-decoder` | 実装済み | CLI のみ | pending #0010 | パース後未使用 |
| `--av1-encoder` / `--av1-decoder` | 実装済み | CLI のみ | pending #0010 | パース後未使用 |
| `--h264-encoder` / `--h264-decoder` | 実装済み | 実装済み | - | videotoolbox 指定に対応 (macOS/iOS)。Sora モードで VideoCodecPreference に設定 |
| `--h265-encoder` / `--h265-decoder` | 実装済み | 実装済み | - | videotoolbox 指定に対応 (macOS/iOS)。Sora モードで VideoCodecPreference に設定 |
| `--video-codec-engines` | 実装済み (利用可能コーデック一覧表示) | 実装済み | - | sora_sdk の VideoCodecCapability を使用して一覧表示 |
| `--openh264` | 実装済み | 実装済み | - | OpenH264 ソフトウェア H.264 エンコーダー/デコーダー。動的ライブラリパスを指定して利用 |
| `--hw-mjpeg-decoder` | 実装済み | CLI のみ | pending #0010 | パース後未使用 |
| H.264 HW エンコード (V4L2) | 実装済み | 実装済み | - | `--use-v4l2-encoder` (raspberrypi feature) |
| H.264 HW エンコード (Jetson) | 実装済み (USE_JETSON_ENCODER) | 未実装 | #0011 | |
| H.264 HW エンコード (NVIDIA) | 実装済み (USE_NVCODEC_ENCODER/CUDA) | 未実装 | #0012 | |
| H.264/H.265 HW エンコード (Intel) | 実装済み (USE_VPL_ENCODER/oneVPL) | 未実装 | #0012 | |
| H.264/H.265 HW エンコード (VideoToolbox) | 実装済み (macOS) | 実装済み | - | `--h264-encoder videotoolbox` / `--h265-encoder videotoolbox` で明示指定。sora_sdk の VideoCodecPreference で設定 |

### TODO

- [x] `--h264-encoder` / `--h264-decoder` の実装 (videotoolbox 指定)
- [x] `--h265-encoder` / `--h265-decoder` の実装 (videotoolbox 指定)
- [x] `--video-codec-engines` の実装
  - sora_sdk の InternalVideoCodecCapability / InternalHwaVideoCodecCapability を使用
  - 各コーデック (VP8/VP9/AV1/H264/H265) のエンコーダー/デコーダー対応状況を表示
- [ ] `--vp8-encoder/decoder`, `--vp9-encoder/decoder`, `--av1-encoder/decoder` の実装 (pending #0010)
  - HW バックエンド (#0011/#0012) が未実装のため pending
- [ ] `--hw-mjpeg-decoder` の実装 (pending #0010)
- [x] `--openh264` の実装 (ソフトウェア H.264 エンコーダー/デコーダー)
  - shiguredo_openh264 crate による Cisco OpenH264 ソフトウェアコーデックの統合
  - HW エンコーダーが利用できない環境での H.264 対応手段
  - P2P / Ayame / Sora 全モードで利用可能
- [ ] H.264 HW エンコード (Jetson) の実装 (#0011)
  - 現状: 未実装。momo は USE_JETSON_ENCODER で H.264/H.265 対応
- [ ] H.264 HW エンコード (NVIDIA) の実装 (#0012)
  - 現状: 未実装。momo は USE_NVCODEC_ENCODER/CUDA で H.264/H.265 対応
- [ ] H.264/H.265 HW エンコード (Intel) の実装 (#0012)
  - 現状: 未実装。momo は USE_VPL_ENCODER/oneVPL で H.264/H.265 対応
- [x] H.264/H.265 HW エンコード (VideoToolbox)
  - shiguredo_webrtc が標準で VideoToolbox を使用するため追加実装不要

### momo のエンコーダー/デコーダー実装詳細

- **OpenH264** (`--openh264`): Cisco OpenH264 動的ライブラリによるソフトウェア H.264 エンコード/デコード
- **Jetson** (USE_JETSON_ENCODER): NVIDIA Jetson ハードウェアエンコーダー (H.264/H.265)
- **NVIDIA** (USE_NVCODEC_ENCODER): CUDA 対応 GPU の NvCodec エンコーダー/デコーダー (H.264/H.265)
- **Intel** (USE_VPL_ENCODER): oneVPL (旧 Intel Media SDK) エンコーダー (H.264/H.265)
- **VideoToolbox** (macOS): Apple VideoToolbox フレームワーク (H.264/H.265)
- **V4L2** (USE_V4L2_ENCODER): Raspberry Pi V4L2 M2M ハードウェアエンコーダー (H.264)
- エンコーダー選択: `--{codec}-encoder` で `default` / `software` / HW バックエンド名を指定
- `--video-codec-engines`: 利用可能な全エンコーダー/デコーダーの組み合わせを一覧表示

### momo-rs のエンコーダー/デコーダー実装詳細

#### OpenH264 ソフトウェアエンコーダー/デコーダー (`--openh264`)

- Cisco OpenH264 によるソフトウェア H.264 エンコード/デコード
- HW エンコーダーが利用できない環境 (x86_64 Linux、macOS 等) での H.264 対応手段
- shiguredo_openh264 crate で OpenH264 動的ライブラリを実行時に読み込む
- `--openh264 <PATH>` で共有ライブラリパスを指定 (macOS: `.dylib`, Linux: `.so`, Windows: `.dll`)
- **エンコーダー**: `VideoEncoderFactoryHandler` を実装し、H.264 は OpenH264 で処理
- **デコーダー**: `VideoDecoderFactoryHandler` を実装し、H.264 は OpenH264 で処理
- **Sora**: `VideoCodecCapability` を実装し、エンコード/デコード両方に対応
- H.264 Constrained Baseline Profile (42e01f) を優先
- H.264 以外のコーデック要求時はビルトインエンコーダー/デコーダーにフォールバック
- エンコーダー: init_encode / encode / release / set_rates コールバック実装
- デコーダー: configure / decode / release コールバック実装
- キーフレーム要求 (force IDR) 対応、ビットレート動的変更対応
- I420 (YUV 4:2:0 planar) 入出力、ストライド変換対応
- 優先順位: `--use-v4l2-encoder` (HW) > `--openh264` (SW) > ビルトイン

#### V4L2 (`--use-v4l2-encoder`)

- shiguredo_v4l2 の `H264Encoder` を使用した V4L2 M2M ハードウェアエンコーディング (raspberrypi feature)
- `VideoEncoderFactory` のカスタム実装で WebRTC に統合
- H.264 Constrained Baseline Profile (42e0) を優先
- H.264 以外のコーデック要求時はビルトインソフトウェアエンコーダーにフォールバック
- init_encode / encode / release / set_rates コールバック実装
- キーフレーム要求対応、ビットレート動的変更対応

## libcamera (Raspberry Pi)

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| `--use-libcamera` | 実装済み | 実装済み | - | raspberrypi feature で有効化 |
| `--use-libcamera-native` | 実装済み (DMA-BUF ネイティブバッファをエンコーダに直接渡す) | 実装済み | #0013 | DMA-BUF fd を V4L2 M2M エンコーダーにゼロコピーで渡す。simulcast 時は無効 |
| `--libcamera-control KEY=VALUE` | 実装済み (キー/バリュー形式、momo は `KEY VALUE` の2引数形式) | 実装済み | #0013 | momo-rs は `KEY=VALUE` 形式。複数回指定可能 |

### TODO

- [x] `--use-libcamera-native` の実装 (#0013)
- [x] `--libcamera-control KEY=VALUE` の実装 (#0013)

### momo の libcamera 実装詳細

- sora-cpp-sdk の `LibcameraCapturer` を使用
- `--use-libcamera-native`: DMA-BUF ネイティブバッファをエンコーダに直接渡す (ゼロコピー)
  - I420Buffer 変換をスキップしてメモリコピーを削減
- `--libcamera-control KEY VALUE`: libcamera のコントロールパラメータを任意に設定
  - 露出、ホワイトバランス、明るさ等のカメラ制御

### momo-rs の libcamera 実装詳細

- shiguredo_libcamera を使用した独自実装 (`src/libcamera.rs`)
- CameraManager → カメラ取得 → StreamRole::VideoRecording でストリーム設定
- YU12 (I420) ピクセルフォーマット固定
- FrameBufferAllocator によるフレームバッファ管理
- DMA-BUF を mmap して読み取り (通常モード)
- `--use-libcamera-native`: DMA-BUF fd を V4L2 M2M エンコーダーにゼロコピーで渡す
  - タイムスタンプをキーにした共有マップで libcamera スレッド → エンコードスレッド間の fd 受け渡し
  - `--use-libcamera` + `--use-v4l2-encoder` が前提、simulcast 時は自動無効化
  - エンコード完了を待機してからバッファを requeue (DMA-BUF の安全な再利用)
- TimestampAligner で WebRTC タイムスタンプに変換
- AdaptedVideoTrackSource の `adapt_frame()` でフレームスケーリング対応
- キャプチャは専用スレッドで実行、`AtomicBool` で停止制御
- Drop 時にキャプチャスレッドを自動停止
- `--libcamera-control KEY=VALUE`: コントロールパラメータの設定 (複数回指定可能)
  - 対応型: Bool, Int32, Int64, Float, Rectangle, 各配列型
  - 主要 enum の文字列指定に対応 (AfMode, AwbMode, AeMeteringMode 等)
  - momo は `KEY VALUE` (2引数) 形式、momo-rs は `KEY=VALUE` (1引数) 形式
  - Request ごとにコントロールを適用 (reuse 後も再適用)
  - 未知のコントロール名やパース失敗時は警告を出力して他のコントロールは適用継続

## ネットワーク・セキュリティ

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| `--no-google-stun` | 実装済み | CLI のみ | - | パース後未使用 (`_no_google_stun`) |
| `--insecure` | 実装済み | 実装済み | #0021 | Ayame: rustls NoVerifier、Sora: sora_sdk insecure + turn_tls_insecure |
| `--cacert` | **なし** | 実装済み | #0026 | CA 証明書指定 (PEM)。momo にはない momo-rs 独自機能 |
| `--proxy-url` | 実装済み (CONNECT トンネリング + Basic Auth) | CLI のみ | #0014 | Sora モードのみ対応予定。sora_sdk の ProxyInfo API を使用。P2P / Ayame は非対応 |
| `--proxy-username` | 実装済み | CLI のみ | #0014 | Sora モードのみ対応予定。P2P / Ayame は非対応 |
| `--proxy-password` | 実装済み | CLI のみ | #0014 | Sora モードのみ対応予定。P2P / Ayame は非対応 |
| `--client-cert` | 実装済み (PEM, TLS 1.2/1.3) | 実装済み | #0015 | Ayame: rustls + rustls-pemfile、Sora: sora_sdk |
| `--client-key` | 実装済み (PEM) | 実装済み | #0015 | --client-cert と同時指定必須 |

### TODO

- [x] `--insecure` の実装 (#0021)
- [x] `--cacert` の実装 (#0026)
- [ ] `--proxy-url` / `--proxy-username` / `--proxy-password` の実装 (#0014)
  - Sora モードのみ対応。sora_sdk の `ProxyInfo` API に渡す
  - P2P / Ayame モードは非対応
- [x] `--client-cert` の実装 (#0015)
- [x] `--client-key` の実装 (#0015)

### momo のネットワーク・セキュリティ実装詳細

- **プロキシ**: HTTP CONNECT メソッドでトンネリング、Basic 認証対応、WebSocket + SSL 通信に対応
- **クライアント証明書**: PEM 形式の証明書・秘密鍵ファイル読み込み、TLS 1.2/1.3 対応、カスタム SSL 検証 (`ssl_verifier.h`)
- **insecure**: SSL 証明書の検証をスキップ (開発用途)

### momo-rs のネットワーク・セキュリティ実装詳細

#### TLS 証明書検証

WSS (WebSocket over TLS) の証明書検証には **`rustls_platform_verifier`** を使用している。
OS のネイティブ証明書ストアを利用する (macOS: Security.framework、Linux: OpenSSL 証明書ストア)。
webpki-roots (内蔵証明書バンドル) は使っていない。

- **Ayame モード**: `rustls::ClientConfig::with_platform_verifier()` で TLS 接続を構築
- **Sora モード**: sora_sdk 内部で同じく `rustls::ClientConfig::with_platform_verifier()` を使用

#### `--insecure` の実装

- **Ayame モード**: rustls の `dangerous()` API で `NoVerifier` (全証明書を受け入れるダミー検証器) を設定
- **Sora モード**: sora_sdk の `.insecure(true)` で WSS 証明書検証スキップ + `.turn_tls_insecure(true)` で TURN-TLS 証明書検証スキップ

#### TURN-TLS の証明書検証 (Sora モード)

sora_sdk 内部では TURN-TLS の証明書検証に **rustls-webpki** (`rustls-webpki` 0.103) を使用している。
shiguredo_webrtc の `SSLCertificateVerifier` コールバック経由で `TurnTlsCaCertVerifier` を実装し、
`webpki::EndEntityCert::verify_for_usage()` で証明書チェーンを検証する。

- `turn_tls_ca_cert` 指定時: DER 形式の CA 証明書を `webpki::anchor_from_trusted_cert()` で `TrustAnchor` としてロードし、`PeerConnectionDependencies::set_tls_cert_verifier()` に設定する
- `turn_tls_insecure` 指定時: WebRTC の ICE サーバー設定で `TlsCertPolicy::InsecureNoCheck` を設定する

#### `--client-cert` / `--client-key` の実装

- PEM ファイルを読み込み、`(cert_pem, key_pem)` タプルとして各モードに伝搬
- `--client-cert` と `--client-key` は同時指定必須 (片方のみはエラー)
- **Ayame モード**: `rustls_pki_types` で PEM をパースし、`with_client_auth_cert()` または `SingleCertResolver` で rustls に設定
- **Sora モード**: `sora_sdk::SoraClientBuilder::client_cert(cert_pem, key_pem)` で設定
- `rustls-pki-types` v1 を依存に追加 (ayame feature で有効化)

#### `--cacert` の実装

- momo にはないオプションのため momo-rs 独自機能
- PEM 形式の CA 証明書ファイルパスを指定
- **Ayame モード**: `RootCertStore` に PEM をロードし `with_root_certificates()` で設定 (platform verifier を使わない)
- **Sora モード**: `sora_sdk::SoraClientBuilder::ca_cert(ca_pem)` で設定
- `--insecure` と同時指定時は `--insecure` が優先 (`--cacert` は無視)

## データチャネル

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| シリアルデータチャネル | 実装済み (Boost ASIO 非同期シリアル通信、DataChannel パススルー) | 実装済み | #0016 | P2P / Ayame モードで利用可能。Linux 限定 |

### TODO

- [x] シリアルデータチャネルの実装 (#0016)
  - `--serial DEVICE,BAUDRATE` でシリアルポートを開き、"serial" ラベルの DataChannel と双方向ブリッジ
  - std::fs + libc termios + tokio AsyncFd で実装 (外部依存なし)
  - P2P / Ayame モードで利用可能。Linux 限定

### momo のシリアルデータチャネル実装詳細

- `SerialDataManager`: シリアルポートの管理と DataChannel の紐付け
- `SerialDataChannel`: WebRTC DataChannel を通じたシリアルデータのパススルー
- Boost ASIO ベースの非同期シリアル通信
- DataChannel でバイナリデータを受信 → シリアルポートに書き込み
- シリアルポートからデータ受信 → DataChannel で送信
- `--serial DEVICE,BAUDRATE` 形式 (例: `--serial /dev/ttyUSB0,9600`)
- P2P / Ayame モードで利用可能

### momo-rs のシリアルデータチャネル実装詳細

- `src/serial.rs` に実装。Linux 限定 (`#[cfg(target_os = "linux")]`)
- シリアルポートの開設: `std::fs::OpenOptions` + `libc::tcsetattr` / `cfmakeraw` / `cfsetspeed` で raw モード設定
- 非同期化: `libc::fcntl` で `O_NONBLOCK` 設定後、`tokio::io::unix::AsyncFd` でラップ
- DataChannel 受信: `DataChannelObserverHandler::on_message` → `mpsc::UnboundedSender` 経由で tokio タスクに転送 → シリアルポートに書き込み
- シリアルポート受信: `AsyncFd::readable()` で非同期待ち → `DataChannel::send()` でバイナリ送信
- ブラウザ側が "serial" ラベルの DataChannel を作成し、momo-rs は `on_data_channel` コールバックで受け取る
- 対応ボーレート: 1200 / 2400 / 4800 / 9600 / 19200 / 38400 / 57600 / 115200 / 230400 / 460800 / 500000 / 576000 / 921600 / 1000000 / 1500000 / 2000000
- 外部依存なし (tokio-serial 等を使わず libc + tokio AsyncFd で実装)

## 表示

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| SDL レンダラー | 実装済み (`--use-sdl`, SDL3, 受信映像表示) | 実装済み (送信プレビューのみ) | #0020 | momo-rs は raw_player crate (SDL3 ベース) で Sora sendonly 時のプレビュー表示。受信映像表示は未実装 |
| `--player` | なし (momo は `--use-sdl`) | 実装済み | - | player feature (デフォルト有効)。Sora sendonly 時にキャプチャ映像を SDL3 プレビュー表示 |
| `--fullscreen` | 実装済み | CLI のみ | #0020 | パース後未使用 |
| `--window-width` / `--window-height` | 実装済み | 実装済み | - | プレビューウィンドウサイズ指定 (デフォルト 640x480) |

### TODO

- [ ] 受信映像表示の実装 (#0020)
  - 現状: 送信プレビュー (Sora sendonly) のみ対応
  - momo は `--use-sdl` で recvonly/sendrecv 時に受信映像を表示可能
- [ ] `--fullscreen` の実装 (#0020)
  - 現状: CLI のみ。パース後未使用

### momo の表示実装詳細

- SDL レンダラー: SDL3 を使用した受信映像の表示ウィンドウ
- `--window-width` / `--window-height`: ウィンドウサイズ指定 (デフォルト 640x480)
- `--fullscreen`: フルスクリーン表示
- `--use-raw-player`: Raw プレイヤーでの映像表示 (SDL の代替)

### momo-rs の表示実装詳細

- raw_player crate (SDL3 ベース) を使用。player feature (デフォルト有効)
- `--player`: Sora sendonly 時にキャプチャ映像を SDL3 プレビューウィンドウで表示
  - メインスレッドで SDL3 イベントループを実行、Sora 接続は別タスクで起動
  - I420 フレームを bounded channel (容量 2) でベストエフォート転送
  - `--window-width` / `--window-height` でウィンドウサイズ指定 (デフォルト 640x480)
- 受信映像表示 (recvonly/sendrecv) は未実装

## ログ・デバッグ

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| `--log-level` | 実装済み (verbose/info/warning/error/none) | 実装済み | #0017 | tracing_subscriber EnvFilter で適用 |
| ログファイル出力 | 実装済み (FileRotatingLogSink, 10MB x 10 ファイル、`webrtc_logs_*`) | 未実装 | #0017 | stdout のみ |
| `--list-devices` | 実装済み | 実装済み | - | JSON 形式でデバイス一覧表示 |

### TODO

- [x] `--log-level` の実装 (#0017)
- [ ] ログファイル出力の実装 (#0017)
  - 現状: 未実装。stdout のみ
  - momo: FileRotatingLogSink, `webrtc_logs_*`, 10MB x 10 ファイル

### momo のログ実装詳細

- `--log-level`: verbose / info / warning / error / none の 5 段階
- ファイル出力: `webrtc::FileRotatingLogSink` でカレントディレクトリに出力
  - ファイル名プレフィックス: `webrtc_logs`
  - 最大ファイルサイズ: 10MB
  - ローテーション: 最大 10 ファイル

### momo-rs のログ実装詳細

- `init_tracing()` でサブコマンド処理前に `tracing_subscriber` を初期化
- `--log-level` を `tracing_subscriber::EnvFilter` に変換 (verbose=TRACE, info=INFO, warning=WARN, error=ERROR, none=OFF)
- ファイル出力の仕組みは未実装

### momo-rs のデバイス一覧実装詳細

- `--list-devices`: shiguredo_video_device と shiguredo_audio_device で列挙
- JSON 形式で出力 (nojson クレート使用)
- ビデオデバイス: name, unique_id, formats (width, height, min_fps, max_fps, pixel_format)
- オーディオ入力デバイス: name, unique_id, channels, sample_rate

## バージョン情報

| 機能 | momo | momo-rs | issue | 備考 |
|------|------|---------|-------|------|
| バージョン番号 | 実装済み (MOMO_VERSION) | 実装済み (CARGO_PKG_VERSION) | - | |
| コミットハッシュ | 実装済み (MOMO_COMMIT_SHORT) | 実装済み | #0018 | build.rs で `git rev-parse --short HEAD` |
| libwebrtc バージョン | 実装済み (WEBRTC_READABLE_VERSION + BUILD_VERSION) | 実装済み | #0018 | `Shiguredo-Build {shiguredo_webrtc::version()}` |
| 環境情報 | 実装済み (OS 名/バージョン/アーキテクチャ、Jetson L4T バージョン) | 実装済み | #0018 | `[ARCH] OS_DETAIL` 形式 |
| ビルドフラグ | 実装済み (USE_JETSON_ENCODER, USE_NVCODEC_ENCODER, USE_V4L2_ENCODER, USE_VPL_ENCODER) | 実装済み | #0018 | Cargo features (ayame, sora, raspberrypi, player) を表示 |

### TODO

- [x] `--version` にコミットハッシュを追加 (#0018)
- [x] `--version` に libwebrtc バージョンを追加 (#0018)
- [x] `--version` に環境情報を追加 (#0018)
- [x] `--version` にビルドフラグを追加 (#0018)

### momo の `--version` 出力例

```
WebRTC Native Client Momo 2024.1.0 (abc1234)
WebRTC: Shiguredo-Build M120 (6099.0 def5678)
Environment: [aarch64] Ubuntu 22.04.3 LTS (nvidia-l4t-core 35.4.1-20230801210015)
```

- 1 行目: momo バージョン + コミットハッシュ
- 2 行目: libwebrtc のビルドバージョン (Shiguredo-Build + Readable Version + Build Version + Commit Short)
- 3 行目: 環境情報
  - Windows: `[x64] Windows 10.0 Build 19045`
  - macOS: `[arm64] macOS 14.2`
  - Linux: `[aarch64] Ubuntu 22.04.3 LTS`
  - Jetson: 上記に加えて `(nvidia-l4t-core バージョン)` を追加
- コンパイルオプション表示: USE_JETSON_ENCODER, USE_NVCODEC_ENCODER, USE_V4L2_ENCODER, USE_VPL_ENCODER

### momo-rs の `--version` 出力

```
shiguredo_momo 2026.0.0 (abc1234)
WebRTC: Shiguredo-Build 0.146.0-canary.4
OpenH264: v2.6.0 (build)
Environment: [aarch64] macOS 15.3
Build Flags: ayame, sora, player
```

- 1 行目: パッケージ名 + バージョン + コミットハッシュ (build.rs で埋め込み)
- 2 行目: shiguredo_webrtc のクレートバージョン
- 3 行目: shiguredo_openh264 のビルド時 OpenH264 バージョン
- 4 行目: `[ARCH] OS_DETAIL` 形式の環境情報
- 5 行目: 有効な Cargo features (build.rs で `CARGO_FEATURE_*` 環境変数から判定)。デフォルト: ayame, sora, player

## E2E テスト

| テストファイル | テスト数 | カバー機能 |
|---------------|---------|-----------|
| `test_p2p_mode.py` | 4 | P2P モード起動、カスタム引数、マルチインスタンス並行動作、動的生成・削除 |
| `test_ayame_mode.py` | 5 | Ayame モード起動、client_id、ビデオ/オーディオ設定、無効コーデックエラー |
| `test_momo_validation.py` | 4 | モード間オプション混在エラー、共通オプション |
| `test_sora_mode_apple_video_toolbox.py` | 6 | Apple VideoToolbox H264/H265 エンコーダー/デコーダー、sendonly/recvonly ペア、simulcast (skip) |

### TODO

実装済み:
- [x] P2P モード起動・メトリクス確認・複数インスタンス (test_p2p_mode.py)
- [x] Ayame モード起動・設定バリエーション・不正コーデック検証 (test_ayame_mode.py)
- [x] モード固有オプションの検証 (test_momo_validation.py)
- [x] momo.py: Momo クラス、get_metrics()、wait_for_connection()、wait_stats 対応
- [x] Apple Video Toolbox E2E テスト (test_sora_mode_apple_video_toolbox.py)
  - H264/H265 VideoToolbox エンコーダーで sendonly 接続、encoderImplementation 確認
  - sendonly/recvonly ペアで encoderImplementation/decoderImplementation が VideoToolbox であることを確認
  - simulcast テストは webrtc-rs が HWA での simulcast 未対応のためスキップ
  - GitHub Actions の self-hosted runner (macOS ARM64) で CI 実行

未実装:
- [ ] Sora モード E2E テスト (#0001)
- [ ] WebRTC 接続確立テスト (#0022)
  - `wait_for_connection()` は `momo.py` に定義済みだが未使用
- [ ] メトリクス API 統計情報詳細検証 (#0023)
  - `find_stats()` / `find_all_stats()` は `test_ayame_mode.py` に定義済みだが未使用
- [ ] VP9 / AV1 / H264 / H265 コーデックの実動作テスト (#0024)
  - VP8 ソフトウェアエンコーダのみテスト済み
- [ ] `--insecure` / 音声処理オプションの動作テスト (#0025)
  - 各機能 issue の実装が前提

### E2E テスト基盤 (`momo.py`)

- `Momo` クラス: プロセス管理 (起動/停止/クリーンアップ)
- コンテキストマネージャー (`with Momo(...) as m:`) でライフサイクル管理
- バイナリ自動検出: 環境変数 `MOMO_BINARY` → `target/debug/momo` → `target/release/momo`
- 起動確認: メトリクス API (`GET /metrics`) へのポーリングで起動完了を検出 (タイムアウト 30 秒)
- グレースフルシャットダウン: SIGTERM → 5 秒待機 → SIGKILL
- `_validate_mode_options()`: モード固有オプションの混在をプロセス起動前に検出
- `get_metrics()`: メトリクス API からデータ取得
  - `wait_stats` パラメータで特定の統計情報が出現するまでポーリング可能
  - stats の type とフィールド値でマッチング
- `wait_for_connection()`: DTLS/ICE 接続確立を stats 経由で待機
  - transport type の `dtlsState: "connected"` と `iceState: "connected"` をチェック
- ポート管理: `conftest.py` で `itertools.count(56000)` による一意なポート割り当て

### test_p2p_mode.py の詳細

| テスト名 | 検証内容 |
|---------|---------|
| `test_with_custom_arguments` | `--resolution QVGA --framerate 15 --log-level info` でプロセス起動、メトリクス version 確認 |
| `test_multiple_instances_concurrent` | 異なるメトリクスポート・HTTP ポートで 2 インスタンス同時起動、各メトリクス独立確認 |
| `test_multiple_instances_different_configs` | 同一設定 (QVGA, 15fps) で 2 インスタンス、ポート競合なしで独立動作確認 |
| `test_dynamic_instance_creation_and_cleanup` | 3 インスタンスを動的生成 → 各メトリクス取得 → 手動クリーンアップ |

### test_ayame_mode.py の詳細

| テスト名 | 検証内容 |
|---------|---------|
| `test_ayame_mode_basic` | UUID room_id で Ayame Labo (`wss://ayame-labo.shiguredo.app/signaling`) に接続、メトリクス version 確認 |
| `test_ayame_mode_with_client_id` | UUID client_id を指定して Ayame Labo に接続 |
| `test_ayame_mode_with_video_settings` | `--vp8-encoder software` を指定して起動確認 |
| `test_ayame_mode_with_audio_settings` | echo cancellation / AGC / noise suppression を全て無効化して起動確認 |
| `test_ayame_mode_with_invalid_codec` | `INVALID_CODEC` / `INVALID_AUDIO` 指定でプロセス異常終了確認 (`RuntimeError`) |

- ヘルパー関数 `find_stats()` / `find_all_stats()` 定義済みだが現テストでは未使用

### test_momo_validation.py の詳細

| テスト名 | 検証内容 |
|---------|---------|
| `test_p2p_mode_with_ayame_options_raises_error` | P2P モードで `room_id` / `client_id` 指定時に `ValueError` 発生確認 |
| `test_ayame_mode_with_p2p_options_raises_error` | Ayame モードで `document_root` 指定時に `ValueError` 発生確認 |
| `test_p2p_mode_with_ayame_direction_raises_error` | P2P モードで `direction` 指定時に `ValueError` 発生確認 |
| `test_common_options_allowed_in_all_modes` | P2P モードで `resolution` / `framerate` / `fake_capture_device` 等の共通オプション使用可能確認 |

- プロセスを起動せずに `_validate_mode_options()` の検証ロジックをテスト (ValueError はコンストラクタで発生)

### E2E テストで未カバーの領域

- Sora モード E2E テスト
- 実際の WebRTC 接続確立 (`wait_for_connection()` は `momo.py` に定義済みだが未使用)
- メトリクス API の統計情報詳細検証 (`find_stats()` / `find_all_stats()` / `wait_stats` は定義済みだが未使用)
- VP9 / AV1 / H264 / H265 コーデックの実動作確認 (VP8 ソフトウェアエンコーダのみテスト)
- シリアルデータチャネル
- プロキシ対応
- クライアント証明書認証 (`--client-cert` / `--client-key` は実装済み、E2E テスト未作成)
- libcamera / V4L2 エンコーダー (Raspberry Pi 専用)
- 音声処理オプション (エコーキャンセレーション等) の実動作 (CLI に渡しているが momo-rs 側で未使用)
- スクリーンキャプチャ
- HW エンコーダー (Jetson / NVIDIA / Intel)
- VideoToolbox simulcast (webrtc-rs が HWA での simulcast 未対応)
- `--metrics-allow-external-ip` の動作検証
- `--insecure` の動作検証
