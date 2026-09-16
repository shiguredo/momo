# 英語コメントを日本語に統一する (AGENTS.md 規約違反の解消)

- Created: 2026-08-19
- Completed: 2026-09-03
- Branch: feature/refactor-japanese-comments
- Polished: 2026-09-02
- Milestone: 2026.1.0

## 目的

AGENTS.md の「コメントは全て日本語にすること」に違反する英語の説明コメントが、boost::beast の example 由来コードと WebRTC / sora-cpp-sdk 由来コードに残っている。規約に沿って日本語へ統一する。

## 現状

`0013-update-sora-cpp-sdk-sdl3-cli11` の sora-cpp-sdk `2026.2.1` 同期は完了している。同期後も英語の説明コメントは残る。`hwenc_jetson` は momo 独自で同期対象外である。

boost::beast サンプル由来（`acceptor_` の open / reuse / bind / listen、セッションの read / write / close など）:

- `src/sora/sora_server.cpp` の `SoraServer` コンストラクタ
- `src/sora/sora_session.cpp` の `SoraSession::DoRead` / `OnRead` / `OnWrite` / `DoClose`
- `src/p2p/p2p_server.cpp` の `P2PServer` コンストラクタ
- `src/p2p/p2p_session.cpp` の `P2PSession::Run` / `DoRead` / `HandleRequest`（一部は既に日本語）
- `src/metrics/metrics_server.cpp` の `MetricsServer` コンストラクタ
- `src/metrics/metrics_session.cpp` の `MetricsSession::Run` / `DoRead`

ヘッダ側にも同系統がある（`src/sora/sora_session.h` の `SoraSession::SendResponse`）。p2p / metrics のセッションヘッダも同系統を確認する。

`screen_video_capturer.cpp` や `sora_client.cpp` の英語は、著作権ブロック、インクルード区切り、コメントアウトされた実行コードが中心である。説明コメントの翻訳対象ではない。コメントアウトの削除は `0038-remove-dead-code`。`util.cpp` の `Util::ShowVideoCodecs` にある `// VP8:` 等は CLI 英語出力の見本であり、ログ文字列と同じく対象外とする。

WebRTC / sora-cpp-sdk 由来:

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_v4l2_capturer.cpp`
- `src/sora-cpp-sdk/src/v4l2/v4l2_video_capturer.cpp`（例: デバイスを開く手順）
- `src/sora-cpp-sdk/src/open_h264_video_encoder.cpp`（`InitEncode` など。`//` の大半が英語）
- `src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_encoder.cpp`（日本語と英語が混在。例: 幅が 16 の倍数である旨）

## 設計方針

- 対象は説明用コメント（`//` / `/* */` で実装の意図を述べるもの）に限る
- 次は対象外とする
  - 著作権・ライセンスのヘッダブロック
  - ログメッセージ（英語必須）と、ログに出す英語文字列、およびその出力見本のコメント
  - コメントアウトされた実行コード（削除は `0038-remove-dead-code`）
  - インクルード区切りの `// WebRTC` / `// Boost`（固有名詞のラベル）
  - `src/sora-cpp-sdk/third_party/`（上流 SDK・ベンダーコード）
- 作業順は momo 固有（`src/sora/` / `src/p2p/` / `src/metrics/` / `src/rtc/` 等、`sora-cpp-sdk` 以外）を先にし、続けて `src/sora-cpp-sdk/` のうち `third_party` を除く残件（`hwenc_jetson` を含む）を日本語化する
- `src/sora-cpp-sdk/` への修正は vendored 差分として残る。次回上流同期で上書きされうる
- 意味を変えずに訳す。コメントアウトコードを復活させない

## 完了条件

- 対象範囲（上記の対象外を除く。`src/sora-cpp-sdk/third_party/` は含まない）に英語の説明コメントが残っていない
- ログメッセージは英語のままである
- コメントアウトされた実行コードは本 issue では削除も復活もしていない

## 解決方法

説明用コメントを日本語に訳した。momo 固有では beast 由来の acceptor / セッション処理、`fake_audio_capturer.h` のセクション見出し、`device_video_capturer` を対象にした。`sora-cpp-sdk` では `third_party` を除き、Jetson / V4L2 キャプチャ、VPL、NvCodec、`open_h264_video_encoder.cpp`、`scalable_track_source.cpp` を対象にした。レビュー指摘で「アライン用」を「アライメント用」に、`kbit/秒` を `kbit/s` に直した。著作権ブロック、ログ文字列、CLI 見本、インクルード区切りラベル、コメントアウトされた実行コードは変更していない。
