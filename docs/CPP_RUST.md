# momo C++ 版と Rust 版の機能比較

このドキュメントは、既存 momo（`develop` ブランチ、C++ 実装）と `feature/momo-rs` ブランチ（Rust 実装）の**機能的な差分**をまとめたものです。

## 全般的な傾向

- Rust 版は C++ 版からの完全再実装です。
- 多くの CLI オプションは「パースするが内部では無視」状態になっています。
- ハードウェアエンコーダーは Raspberry Pi V4L2 M2M のみに大幅に縮小しています。
- 音声出力、リモート映像プレビュー、自動再接続、Sora HTTP 制御サーバーなどが未実装です。
- Sora モードは `sora_sdk` クレートへの委譲が進み、momo 側の細かい挙動が減っています。

## 1. CLI / エントリポイント

| 項目 | C++ 版 | Rust 版 | 備考 |
|------|--------|---------|------|
| `--use-sdl` | 有効 | `--player` に改名 | |
| `--fullscreen` | 有効 | パースのみ、未使用 | `_fullscreen` に束縛され使用されない |
| `--screen-capture` | 有効 | パースのみ、未使用 | `_screen_capture` に束縛され使用されない |
| `--hw-mjpeg-decoder` | 有効 | パースのみ、未使用 | `_hw_mjpeg_decoder` に束縛され使用されない |
| `--audio-output-device` | 有効 | パースのみ、未使用 | `_audio_output_device` に束縛され使用されない |
| `--disable-echo-cancellation` 等 | 有効 | パースのみ、未使用 | `_disable_*` に束縛され使用されない |
| `--help-all` | 有効 | なし | `noargs` には未対応 |
| WebRTC ログファイル出力 | 有効 | なし | `tracing` は stdout/stderr のみ |
| `--libcamera-control` 書式 | `key value` | `KEY=VALUE` | |
| `--sora-signaling-urls` | 繰り返し指定可 | カンマ区切り 1 引数 | |
| `--sora-data-channel-signaling` / `--ignore-disconnect-websocket` | `true` / `false` / `none` | `true` / `false` / 未指定（`None`） | 未指定が `none` 相当 |
| 数値範囲チェック | 有り | ほぼなし | `--framerate` 等 |
| `--log-level` 未指定時 | `LS_NONE` | `INFO` | |
| `--log-level` 未知値 | 拒否 | 警告後 `INFO` フォールバック | |
| サブコマンド未指定時 | `exit(1)` | ヘルプ表示後正常終了 | |
| `--document-root` デフォルト | カレントディレクトリ | `html` | P2P モード |
| `--list-devices` 出力 | テキスト | JSON | Rust 版は音声出力デバイスなし |
| `--vp8-encoder` / `--vp9-encoder` / `--av1-encoder` 等 | 有効 | パースのみ、未使用 | `_vp8_encoder` 等に束縛され使用されない |
| `--priority DISABLED` | なし | 追加 | `--fixed-resolution` 指定時はそちらが優先 |
| `--cacert` | なし | 追加 | オプション名は `--cacert`（`--ca-cert` ではない） |
| `--proxy-url` の適用範囲 | 全モード（`RTCManager` 経由） | Sora モードのみ | Rust 版はグローバルにパースするが `SoraConfig` のみに渡す |
| `--version` | Momo 名・バージョン・コミット短縮ハッシュ・WebRTC・Environment・Build Flags | C++ 版に加え OpenH264 バージョンを追加 | C++ 版も Build Flags を出力する |

## 2. Ayame モード

| 項目 | C++ 版 | Rust 版 | 備考 |
|------|--------|---------|------|
| 自動再接続 | 有効（watchdog + 指数バックオフ） | 未実装 | 切断・`bye`・`reject` 時は終了。ICE failed はログ出力のみで無反応 |
| `register` メタデータ | `ayameClient` / `libwebrtc` / `environment` 送信 | `type` / `roomId` / `clientId` / `key` のみ | |
| legacy サーバー対応（`isExistUser` なし） | 対応（`isExistUser` 欠落時も offer 送信） | 未対応 | `isExistUser` 欠落時は `false` 扱いで offer を送信しない |
| `ping` 受信時の watchdog リセット | 有り | なし | `pong` 返信のみ |
| `reject` メッセージ | 無視 | 即エラー終了 | |
| `bye` 後の再接続 | 再接続 | 即終了 | |
| answer 側の `set_codec_preferences` | 有り | 未実装 | answer 側では Offer 側の `set_codec_preferences` に委ねる |
| ICE 状態による stats 取得制御 | 有り | なし | |
| `--cacert` | なし | 有り | |
| シリアル DataChannel | 有り（全モード共通） | 有り（Linux のみ、P2P/Ayame のみ） | C++ は `RTCDataManagerDispatcher` 経由で全モード対応 |
| `client_id` 未指定時 | ランダム生成 | ランダム生成 | 生成ルールが異なる |
| コーデック入力バリデーション | 有り | 有り | 両方ともバリデーションする |

## 3. P2P モード

| 項目 | C++ 版 | Rust 版 | 備考 |
|------|--------|---------|------|
| SDL レンダラー / プレーヤー | 有効 | 未実装 | `--use-sdl`, `--fullscreen` 等 |
| 画面キャプチャ | 有効 | 未実装 | |
| HW MJPEG デコーダ | 有効 | 未実装 | |
| 個別コーデックエンジン選択 | 有効 | パースのみ | `--vp8-encoder` 等 |
| シミュルキャスト | 有効 | 未実装 | |
| アプリケーションレベル `ping` | 30 秒間隔送信 | なし | |
| HTTP `Server` ヘッダー・keep-alive | 有り | なし | |
| `--insecure` / `--proxy-url` 反映 | P2P モードでは実質未使用 | 未使用 | C++ 版も P2P サーバー動作では機能しない |
| `--video-codec-type` / `--audio-codec-type` | なし | 有効 | `transceiver.set_codec_preferences()` に適用 |
| `--priority DISABLED` | なし | 値はパースされるが P2P では未適用 | `DegradationPreference` は計算されるが `RtpSender::set_degradation_preference()` を呼ばない |
| `--use-libcamera-native` + `--use-v4l2-encoder` | なし | 有効 | |
| `--force-i420` / `--force-yuy2` / `--force-nv12` | なし | 有効（相互排他チェック付き） | |
| `--openh264` | 有効 | 有効 | 両方とも OpenH264 動的ロードに対応 |
| `register` 応答の `isExistUser` | 常に `true` | 実際の PeerConnection 有無 | |
| メトリクス stats プロバイダー | 最新 1 セッションのみ | WebSocket セッションごと | |
| `--fixed-resolution` に相当する `content_hint` 設定 | 有り | 未実装 | |

## 4. Sora モード

| 項目 | C++ 版 | Rust 版 | 備考 |
|------|--------|---------|------|
| `--spotlight-number` | 有効 | なし | Sora クライアント側の `--spotlight-number` は非推奨のため、Rust 版ではパースもしない |
| `--port` / `--auto`（HTTP 制御サーバー） | 有効 | パースのみ | `/connect`, `/close`, `/mute`, status API 未実装 |
| `--data-channel-signaling-timeout` | 有効 | パースのみ | |
| notify メッセージの詳細なロギング | 有り | raw JSON のみ | |
| DataChannel 圧縮ラベル管理 | 有り | `sora_sdk` 委譲 | |
| 明示的な reconnect ロジック | 有り | `sora_sdk` 委譲 | |
| Redirect ハンドリング | 明示的 | `sora_sdk` 委譲 | |
| `connect` メッセージの識別情報 | momo 自身で付加 | `sora_sdk` 委譲 | |
| `--data-channel-signaling` / `--ignore-disconnect-websocket` | 実装 | `sora_sdk` 委譲 | `ignore_disconnect_websocket=true` は現状 xfail |
| サイマルキャスト encoding パラメータ | 手動パース | `builder.simulcast(true)` 委譲 | |
| `--player` | リモート映像を描画 | Sora 送信時のローカルプレビューのみ | C++ 版も Sora モードでローカル映像は描画しない |
| `--openh264` による capability 登録 | オプションは存在 | Openh264VideoCodecCapability を明示登録 | C++ 版は capability 登録を行わない |
| `--audio-codec-type` 受け付け値 | `OPUS` のみ | `OPUS` のみ | 両方とも Sora モードでは OPUS のみ（P2P/Ayame では PCMU/PCMA も受け付ける） |

## 5. メトリクス API

| 項目 | C++ 版 | Rust 版 | 備考 |
|------|--------|---------|------|
| keep-alive HTTP セッション | 有り | なし | 1 リクエストで終了 |
| `Server` レスポンスヘッダー | 有り | なし | |
| `SO_REUSEADDR` 明示設定 | 有り | 未確認 | Rust 版はアプリケーション側で設定しないが、`tokio::net::TcpListener::bind` は Unix 上で暗黙に設定することがある |
| Windows 環境情報 | 詳細（バージョン・ビルド番号） | OS 名のみ | Rust 版は `std::env::consts::OS` のみ |
| Jetson 環境情報 | 有り | 未対応 | `nvidia-l4t-core` バージョン等 |
| バージョン文字列にコミットハッシュ | 有り | なし | |
| libwebrtc フィールド詳細 | `Shiguredo-Build ...` | `webrtc-rs <version>` のみ | |
| ICE 状態による stats 取得制御 | 有り（Ayame のみ） | なし | C++ 版も P2P/Sora では ICE 状態を見ずに取得 |
| 複数 PeerConnection からの stats マージ | なし | 有り | 結合処理は JSON 配列を前提 |
| CORS ヘッダー | なし | 有り | `Access-Control-Allow-Origin: *` |
| async/await + oneshot 収集 | なし | 有り | |
| 死んだプロバイダーの自動除去 | なし | 有り | |
| Ayame モードでの stats 取得 | 取得可能 | 取得可能 | `metrics_state` は `main.rs` → `ayame::run` → `MetricsState::register` → `get_stats` と配線済み |

## 6. シリアル DataChannel

| 項目 | C++ 版 | Rust 版 | 備考 |
|------|--------|---------|------|
| 複数 DataChannel へのブロードキャスト | 有り | 未対応 | 1 DataChannel ごとに独立タスク／ポートを開く |
| シリアル → DataChannel の改行区切り送信 | 有り | 未対応 | raw バイナリ塊を送信 |
| 文字長・パリティ・ストップビット・フロー制御 | 個別設定 | 未対応 | `cfmakeraw()` + ボーレートのみ |
| 16 バイト chunk 書き込み | 有り | 未対応 | 1 メッセージ全体をまとめて書き込み |
| Manager 作成時の接続失敗伝播 | 有り | 未対応 | DataChannel 受信後にポートを開く |
| ボーレートのホワイトリスト検証 | なし | 有り | |
| `--serial` 未設定時の警告 | なし | 有り | |
| Linux 専用コンパイル時ガード | なし | 有り | |
| Sora モードでのシリアル連携 | 有り（全モード共通） | 未対応 | P2P/Ayame のみ呼び出し |

## 7. 映像キャプチャ / コーデック

| 項目 | C++ 版 | Rust 版 | 備考 |
|------|--------|---------|------|
| NVIDIA VIDEO CODEC SDK | 有り | なし | `USE_NVCODEC_ENCODER` |
| Intel VPL | 有り | なし | `USE_VPL_ENCODER` |
| Jetson (L4T Multimedia API) | 有り | なし | `USE_JETSON_ENCODER` |
| macOS / iOS VideoToolbox（P2P/Ayame） | 有り | なし | Sora モードのみ `sora_sdk` 経由 |
| V4L2 H.264 デコーダー | 有り | 未実装 | `create_video_decoder` が常に `None` |
| 画面キャプチャ | 有り | 未実装 | `--screen-capture` パースのみ |
| Direct V4L2 キャプチャ | 有り | なし | `shiguredo_video_device` 抽象レイヤーに置き換え |
| MJPEG ネイティブキャプチャ / HW MJPEG デコード | 有り | 未実装 | |
| サイマルキャスト（P2P/Ayame） | 有り | 未実装 | `simulcast` オプションは Sora モード専用 |
| H264 profile packetization-mode 0/1 両提示 | 有り | mode=1 のみ | |
| OpenH264 サイマルキャスト / temporal layer / マルチスレッド | 有り | 未対応 | 単一ストリームのみ |
| Blend2D による複雑な fake 映像 | 有り | 未対応 | `raden` で簡易描画 |
| `--use-libcamera-native` + DMA-BUF ゼロコピー | 有り | 有り | C++ 版も対応している |
| `--libcamera-control` の豊富なパース | 制限あり | 有り | 主要 enum の文字列解決等 |
| `--force-i420` / `--force-yuy2` / `--force-nv12` 相互排他 | 有り | 有り（明示的チェック） | C++ 版も if-else による排他あり |
| OpenH264 動的ロード失敗時のフォールバック | 失敗 | エラー終了 | ソフトウェアフォールバックはエンコーダー初期化失敗時の挙動 |
| H264 ソフトウェアデコード | 未対応 | `--openh264` で対応 | |

## 8. 音声 / プレビュー / ビルド

### 音声

| 項目 | C++ 版 | Rust 版 | 備考 |
|------|--------|---------|------|
| 音声出力（プレイアウト）デバイス選択 | 有り（Linux/macOS） | **未実装** | `--audio-output-device` 無視 |
| 受信側音声の再生 | 有り | 未実装 | 外部 ADM は録音のみ |
| 音声入力デバイスの index / name / GUID 選択 | index/name/GUID | `device_id` 文字列 | |
| 音声処理無効化オプションの実適用 | 有り | 未実装 | `--disable-*` 無視 |
| 全モードで動作するフェイク音声ビープ | 有り | Sora のみ | P2P/Ayame は無音ダミー ADM |
| F32 → S16 変換 | なし | 有り | `AdmState::on_audio_frame` |

### プレビュー

| 項目 | C++ 版 | Rust 版 | 備考 |
|------|--------|---------|------|
| ローカル・リモート複数トラックのグリッド表示 | 有り | 未対応 | ローカル映像のみ |
| フルスクリーン切替 | 有り | 未対応 | `--fullscreen` 無視 |
| キーボード操作（F / Q） | 有り | 未対応 | |
| ウィンドウクローズでプロセス終了 | 有り | 未対応 | プレビューループのみ終了 |
| P2P / Ayame でのプレビュー | 有り | 未対応 | Sora 送信時のみ |

### ビルド・機能フラグ

| 項目 | C++ 版 | Rust 版 | 備考 |
|------|--------|---------|------|
| `USE_NVCODEC_ENCODER` / `USE_JETSON_ENCODER` / `USE_VPL_ENCODER` | 有り | なし | |
| `USE_SCREEN_CAPTURER` | 有り | なし | |
| Cargo features | なし | `ayame` / `sora` / `player` / `raspberrypi` | `default = ["ayame", "player", "sora"]` |
| `build.rs` の feature 名 | - | `PREVIEW` を参照 | `Cargo.toml` では `player` のため、`MOMO_BUILD_FLAGS` に `player` が含まれない不整合 |

## 未実装・部分的実装が多い領域のまとめ

1. **ハードウェアエンコーダー**：Jetson / NVIDIA / Intel VPL / macOS VideoToolbox（P2P/Ayame）
2. **音声**：出力デバイス選択、音声処理無効化、P2P/Ayame のフェイク音声ビープ
3. **プレビュー**：リモート映像、フルスクリーン、P2P/Ayame 対応
4. **Ayame**：自動再接続、legacy サーバー対応、`register` メタデータ
5. **Sora**：HTTP 制御サーバー、`spotlight-number`、`data-channel-signaling-timeout`
6. **キャプチャ**：画面キャプチャ、HW MJPEG デコード、Direct V4L2
7. **コーデック**：VP8/VP9/AV1 の個別エンコーダー/デコーダー選択、P2P での degradation preference 適用
8. **シリアル**：改行区切りフレーミング、複数 DataChannel ブロードキャスト、Sora モード対応

これらの多くは `issues/pending/` に整理されている項目と一致しています。
