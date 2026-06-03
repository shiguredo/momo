# libwebrtc のバージョンを m150 に上げる

- Priority: Medium
- Created: 2026-06-03
- Model: Opus 4.8
- Branch: feature/change-libwebrtc-m150
- Polished: 2026-06-03

## 現状

- momo の libwebrtc のバージョンは `DEPS` の `WEBRTC_BUILD_VERSION` で管理されており、現在 `m138.7204.0.4`。
- `webrtc-build` は既に `WEBRTC_BUILD_VERSION=150.7871.0.0` をリリース済みであるため、着手可能。
- `sora-cpp-sdk` は `WEBRTC_BUILD_VERSION=m149.7827.0.0` であり、m150 未対応。
  - sora-cpp-sdk 側の `absl::make_unique` 使用箇所（`sora_video_decoder_factory.cpp`, `sora_client_context.cpp`）の対応も必要。
- `run.py` が `DEPS` の `WEBRTC_BUILD_VERSION` を読み取り、`webrtc-build` のリリースを `buildbase.py` の `install_webrtc()` 経由で取得する。

## 目的

libwebrtc のバージョンを現在の `m138.7204.0.4` から m150 系へ更新する。

## 優先度根拠

- Medium とする。
- libwebrtc の追従は momo の根幹となる依存であり技術的重要性は高いが、m138 でも現状の機能は動作しているため緊急性は High ではない。
- 一方で m138 から m150 は milestone 12 個分の差があり、放置するほど API 変更の追従コストが膨らむため、優先的に対応すべき Medium とする。

## 設計方針

### DEPS 更新

- `DEPS` の `WEBRTC_BUILD_VERSION` を `150.7871.0.0` へ更新する。

### sora-cpp-sdk の対応

`src/sora-cpp-sdk/` は `sora-cpp-sdk` リポジトリからコピー管理されており、`copy-from-sora-cpp-sdk.sh` / `copy-to-sora-cpp-sdk.sh` で双方向同期する。以下のいずれかの方針で対応する:

- **A. sora-cpp-sdk 側で先に m150 対応リリースを行い、momo 側でコピーを取り込む**（推奨）
- **B. momo 側のコピーを直接修正し、差分を `copy-to-sora-cpp-sdk.sh` で実体リポジトリに反映する**

着手時に sora-cpp-sdk の m150 対応状況を確認し、未対応の場合は sora-cpp-sdk 側の対応を先行して行うか、上記 B の方針を取るかを判断する。

### API 変更追従

m138 から m150 への更新では、上流の WebRTC API のシグネチャ変更・削除・名前変更が発生している可能性が高いため、ビルドエラーを 1 つずつ解消する。前回（m138）更新時の変更パターンを参考に、`grep` で廃止 API や名前空間変更を systematic に検出する。具体的な確認項目は以下の通り:

- **`absl::make_unique` の削除**: 以下の箇所で使用。`std::make_unique` への置換を実施する。
  - `src/rtc/rtc_manager.cpp:153,195,207`（212 行目は既に `std::make_unique` なので対象外）
  - `src/sora-cpp-sdk/src/open_h264_video_encoder.cpp:953`
  - `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp:628`
  - 上記に加えて、実体の sora-cpp-sdk リポジトリ側にも `absl::make_unique` が存在する（`sora_video_decoder_factory.cpp:147,158,169,180`、`sora_client_context.cpp:65`）。これらは sora-cpp-sdk 対応時に合わせて修正する。
- **`DegradationPreference::MAINTAIN_FRAMERATE` の改名**: m141 以降で `MAINTAIN_FRAME_RATE` に改名されている可能性がある。`src/rtc/rtc_manager.h:83`、`src/momo_args.h:49`、`src/util.cpp:151` の該当箇所を確認する。
- **`RtcEventLogFactory` コンストラクタ**: `src/rtc/rtc_manager.cpp:153` では引数 `&env.task_queue_factory()` 付きで生成しているが、sora-cpp-sdk の `sora_client_context.cpp:65` では引数なしで生成している。m150 でコンストラクタシグネチャが変更された場合、両方の修正が必要になる点に注意する。
- **`PeerConnectionFactoryProxy::Create`**: シグネチャ変更の可能性を確認する。
- **`ConnectionContext`**: `src/rtc/rtc_manager.h:21-48` の private API (`pc/connection_context.h`, `pc/peer_connection_factory.h`) 依存部分。コンストラクタシグネチャ変更の影響を確認する。private API のため、マイナーバージョンでも予告なく破壊的変更が入るリスクがある。可能であれば公開 API への移行を検討するが、本 issue の範囲では既存の private API 依存を維持し、シグネチャ変更に追従する方針とする。
- **`VideoCaptureFactory` / `VideoCaptureModule`**: `src/rtc/device_video_capturer.cpp` の `CreateDeviceInfo()` / `GetDeviceName()` / `NumberOfDevices()` API 変更の可能性を確認する。
- **名前空間の変更**: `webrtc::revive::` プロキシ API の存否確認。`revive_proxy.patch` は `CreateClientTcpSocket()` の純仮想関数シグネチャを変更しているため、m150 で上流がこのメソッドの引数を変更していた場合、手動でのコンフリクト解決が必要になる。

### 影響を受けるファイル

以下のファイルは libwebrtc API を直接利用しており、ビルドエラー発生時に修正対象となる:

- `src/rtc/` 配下の全ファイル
- `src/main.cpp`
- `src/CMakeLists.txt`
- `src/sora-cpp-sdk/src/hwenc_nvcodec/` 配下
- `src/sora-cpp-sdk/src/hwenc_vpl/` 配下
- `src/sora-cpp-sdk/src/hwenc_jetson/` 配下
- `src/sora-cpp-sdk/src/open_h264_video_encoder.cpp`

`src/sora-cpp-sdk/` 配下はコピー管理対象のため、修正後は `copy-to-sora-cpp-sdk.sh` で実体 sora-cpp-sdk リポジトリに差分を反映する。実体 sora-cpp-sdk リポジトリのみに存在するファイル（`sora_video_decoder_factory.cpp`、`sora_client_context.cpp` 等）は sora-cpp-sdk 側で直接修正する。

### sora-cpp-sdk 互換性

- `sora-cpp-sdk` が m150 対応リリースを行っている場合はそのリリースを取り込む。
- 未対応の場合は上記「sora-cpp-sdk の対応」の方針に従い、momo 側で先行対応または sora-cpp-sdk 側で先に対応するかを判断する。

### ツールチェイン

- `webrtc-build` の `webrtc.version` に従い、Clang / libc++ のバージョン追従が必要な場合は対応する。前回 m138 更新時は clang 18 → 20 への更新が発生しており、m150 でも同様の更新が想定される。
- `DEPS` に記載されている他の依存ライブラリ（CUDA, CMake, OpenH264, VPL, Boost, CLI11 等）の最低バージョン要件が上がることがあるため、ビルドエラー発生時に合わせて更新する。参考までに、sora-cpp-sdk（m149 対応時）の DEPS では `BOOST_VERSION=1.91.0`、`VPL_VERSION=v2.16.0`、`CMAKE_VERSION=4.3.2` と momo より新しいバージョンを要求している。

### プラットフォーム

各プラットフォーム（macOS / Ubuntu / Windows / Raspberry Pi / Jetson）でビルドが通ることを確認する。新しい framework やライブラリリンク（前回 m128 更新時の ScreenCaptureKit 相当）が必要な場合は `CMakeLists.txt` に追加する。最初は Ubuntu x86_64 でビルドエラーを一通り解消し、その後他のプラットフォームに展開する順序を推奨する。

## 完了条件

- `DEPS` の `WEBRTC_BUILD_VERSION` が `150.7871.0.0` に更新されている。
- 全対応プラットフォーム（macOS / Ubuntu / Windows / Raspberry Pi / Jetson）で momo がビルドできる。
- libwebrtc の API 変更への追従が完了し、ビルド警告・エラーが解消されている。
- `tests/` 配下の全 E2E テストが通過する。
  - テスト内容: Sora モード、各エンコーダー（NVIDIA / Intel VPL / Raspberry Pi / VideoToolbox / AMF）の送受信、P2P モード、Ayame モードの基本動作。
  - 回帰なしの判断基準: m138 ベースの develop ブランチと同一テストスイートで全ケース PASS すること。
  - E2E テストの実行には Sora / Sora Labo のバックエンドが必要。テスト実行前に CI 上で必要なシークレットや環境変数が設定されていることを確認する。
- `CHANGES.md` の `## develop` セクションに `[UPDATE] WebRTC を m150.7871.0.0 に上げる` を追記し、担当者を明記する（書式: 次の行に `- @担当者名`）。
