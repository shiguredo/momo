# Momo を webrtc m150.7871.0.0 に上げる

- Priority: Medium
- Created: 2026-06-03
- Completed: 2026-06-22
- Model: Opus 4.8
- Polished: 2026-06-03
- Branch: feature/update-libwebrtc-m150 (shiguredo/momo 側のブランチ)

## 目的

shiguredo/momo が依存する libwebrtc を `m150.7871.0.0` へ追従する。`DEPS` の `WEBRTC_BUILD_VERSION` を更新し、M150 の libwebrtc API 変更にコードを追従させ、全プラットフォームでビルド・動作することを確認する。

## 優先度根拠

- Medium とする。
- Momo の M150 提供に必要な定常的なマイルストーン追従。
- 現状 M138 系（後述）にとどまっており、M150 まで 12 マイルストーン分を一気に追従するため作業量が大きい点に注意が必要だが、稼働中バージョンがあるため即時必須の High ではない。

## 現状

- https://github.com/shiguredo/momo/releases/tag/2026.1.0-canary.0 を作成済み
- `shiguredo/momo#455` で `M150` の `PR` はマージ済みである。
- `2026.1.0-canary.0` の GitHub Release は公開済みである。
- `m150` への更新後、ビルドと `e2e`、および手動検証まで完了した。
- `m148` のビルドは通っている。
- `d6826972` で `m148.7778.4.0` へ更新し、`Boost` を `1.91.0` に上げ、`CMake` を `4.3.2` に上げ、`VPL` を `v2.16.0` に上げた。macOS ビルドでは `BOOST_ASIO_DISABLE_STD_ATOMIC_WAIT` を追加し、`webrtc-build` 同梱の `libc++` と組み合わせた `async_connect` のハングを回避するようにした。
- `d1dea570` で `m146.7680.3.1` へ更新した。`SSLCertificateVerifier` は `Verify` から `VerifyChain` に変わり、`PeerConnectionFactory` は `Environment` を受け取る 3 引数構成になり、`AudioDeviceBuffer` も `Environment` 受け取りに変わった。`hwenc_nvcodec` / `hwenc_v4l2` / `hwenc_vpl` の `OnEncodedImage` 失敗時の扱いも `error` 返却から `warning` ログ出力へ変わった。
- M140 のローカルビルド版 `momo` を使って、`P2P` モードの接続確認は完了している。
- M140 への更新は完了した。`m140` への更新作業の一部として、`Xcode` の `clang` ではなく `libwebrtc` 付属の `clang` / `libc++` を使うよう切り替えた。
- 変更の流れは `13ce1700` で libwebrtc を `m140` に上げ、`b6923483` で `sora-cpp-sdk` の `LAST_UPDATED` を更新し、`fb23671d` で macOS の `clang` / `libc++` を `libwebrtc` 管理下のものへ切り替え、`11d32f3f` で `PeerConnectionFactory` の GCM 暗号スイート設定を `RTCConfiguration` 側へ移した、という順になっている。これらはすべて `m140` への更新に伴って実施した変更である。
- M140 では macOS arm64 のリンク時に `Undefined symbols for architecture arm64` が大量に発生している。未解決シンボルに `std::__1::basic_string_view` を含むものがあり、単純な libwebrtc の版差ではなく、`Sora C++ SDK PR #236` で入った Apple libc++ から Chromium libc++ への切り替えと整合していない ABI 不一致が疑わしい。
- Momo は M138 から M150 へ一気に上げると、コード差分だけではビルド結果を完全に読み取れない。ジャンプアップのまま進めると解決すべきハードルが高く、失敗時の影響も大きすぎるため、まずは M140 を通す段階的な進め方に切り替える。
- `sora-cpp-sdk` は `2025.6.0-canary.4` 相当から開始する前提で考える。
- `DEPS` の `WEBRTC_BUILD_VERSION=m138.7204.0.4`。
- Momo は Sora C++ SDK を経由せず、webrtc-build を**直接**参照している（`DEPS` に `SORA_CPP_SDK_VERSION` は無い）。したがって本対応は webrtc-build のタグ打ち（0001）にのみ依存し、Sora C++ SDK（0002）には依存しない。
- `CHANGES.md` の `## develop` には CUDA / オプション名変更など本件とは無関係の変更が積まれている。これらの変更は先にリリースするか、本 issue に含めるか判断してから着手すること。
- 自身のバージョン（`VERSION`）は `2025.2.0-canary.4`。
- M138 → M150 と差分が非常に大きいため、間のマイルストーンで入った libwebrtc の API 変更・ビルド構成変更をまとめて取り込む必要がある。

### 既知のコンパイルエラー箇所

以下の 6 箇所は、コード調査の結果、`WEBRTC_BUILD_VERSION` を M150 に上げた時点で影響を受けることがわかっている。ビルド前に先に修正候補として扱って問題ない。

1. `src/rtc/rtc_ssl_verifier.h:10` - M150 で SSL verifier 周辺の型定義や宣言が変わり、旧シグネチャのままでは整合しない。
2. `src/rtc/rtc_ssl_verifier.cpp:10` - 同じく実装側でも新しい宣言に合わせる必要があり、旧コードのままではコンパイルできない。
3. `src/rtc/rtc_manager.cpp:153` - `ConnectionContext::Create` / `PeerConnectionFactoryProxy::Create` 系の private API を旧い引数構成で呼んでいる。
4. `src/rtc/rtc_manager.cpp:153,195,207` - 同ファイル内で同じ private API 前提の呼び出しが複数箇所にあり、まとめて追従が必要になる。
5. `src/sora-cpp-sdk/src/open_h264_video_encoder.cpp:953` - M150 の `RTC_LOG` / `MakeVal` 変更で、従来の値をそのままログに渡せなくなる。
6. `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp:628` - 同様に `RTC_LOG` の引数型が変わり、旧来のログ出力がそのまま通らない。

### ビルド後に要確認の private API 変更

以下の 4 つは、実際にビルドして初めて影響有無が確定する private API のシグネチャ変更である。

- `ConnectionContext::Create`
- `ConnectionContext::CreateEnvironment`
- `PeerConnectionFactoryProxy::Create`
- `VideoCaptureFactory`

## 依存関係

- 本対応は webrtc-build 側で `m150.7871.0.0` タグが打たれて Release が公開されていることが前提。
- 関連 issue: [0001-release-tag-webrtc-build-m150.md](./0001-release-tag-webrtc-build-m150.md)（webrtc-build のタグ打ち）。0001 完了後に着手する。
- 作業対象リポジトリ: `github.com/shiguredo/momo`。`develop` ブランチから `feature/update-webrtc-m150` ブランチを切って作業する。
- DEPS に `SORA_CPP_SDK_VERSION` はなく、Momo は webrtc-build を直接参照する。ただし `src/sora-cpp-sdk/` 配下に sora-cpp-sdk のソースコードを vendoring しており、libwebrtc API 変更の影響を受ける箇所が含まれる可能性がある。0002 の成果を直接の前提とはしないが、同様の API 追従作業が Momo 側でも発生することを認識しておくこと。

## 設計方針

shiguredo/momo の develop から切ったブランチ上で以下を実施する。

### 1. WEBRTC_BUILD_VERSION の更新

- `DEPS` の `WEBRTC_BUILD_VERSION` をまず `m140` 相当まで更新し、M140 でのビルドを通したうえで M150 へ段階的に追従する。
- webrtc-build の M150 対応で `run.py` の `COMMON_GN_ARGS` に `rtc_rust=false` が追加され、`rtc_rusty_base64=false` が削除されている。Momo 側の `DEPS` や GN 引数設定で同様の変更が必要か確認する。
- 必要に応じて `BOOST_VERSION` / `CMAKE_VERSION` / `VPL_VERSION` 等、M150 の webrtc-build と整合させるべき周辺バージョンも見直す。

### 2. libwebrtc M150 API 追従（コード修正）

M138 → M150 の差分で削除・変更された libwebrtc API に対し、`src/` 配下を追従させる。`src/sora-cpp-sdk/` 配下の vendored コードも含めること。

追従作業は sora-cpp-sdk の M149/M150 対応実績を参考にする。sora-cpp-sdk で実際に必要だった修正は以下の通り:

- **M149**: `webrtc::RtcEventLogFactory` のコンストラクタから `TaskQueueFactory` 引数が削除された。`src/sora_client_context.cpp` で余分な引数を削除する。参考: https://source.chromium.org/chromium/_/webrtc/src/+/987aa57ba46fe759284caa9a2cdad3c52ea5d13a
- **M150**: `MakeVal` の stringstream フォールバックが削除された。`boost::system::error_code` や `tcp::endpoint` を直接 `RTC_LOG` に渡せなくなるため、`ec` は `ec.message()` に、`endpoint` は `address().to_string() + port()` に分解する。参考: https://webrtc-review.googlesource.com/c/src/+/469260

上記の他に M138 → M150 の間で入った API 変更がないか、sora-cpp-sdk の CHANGES.md および git log を横断的に確認し、不足がないか検証する。特に以下は sora-cpp-sdk では影響がなかったが、Momo のコードが異なる実装パターンを持っている場合は別途確認する。
  - `PeerConnectionFactory` / `PeerConnection` のコンストラクタ・メソッドシグネチャ変更
  - `VideoEncoderFactory` / `VideoDecoderFactory` のインタフェース変更
  - `CreateVideoSource` / `CreateVideoTrack` のシグネチャ変更
  - `AudioDeviceModule` の構造変更
  - `SdpObserver` / `CreateSessionDescriptionObserver` / `SetSessionDescriptionObserver` の変更
  - 非推奨 API の削除（12 マイルストーン分の累積削除）

コンパイルエラー・リンクエラーを上流変更に追従する形で順次解消する。追従した上流コミットは `CHANGES.md` に参考リンク付きで記録すること。

### 3. ビルド・動作確認

- 全対象プラットフォーム（Windows x86_64 / macOS arm64 / Ubuntu 22.04 / 24.04 の x86_64、Ubuntu 22.04 armv8（Jetson 含む）、Raspberry Pi OS armv8）で CI のビルドが通ることを確認する。`.github/workflows/` 配下の全ワークフロー（`build.yml`）が成功すること。
- CUDA / VPL 等のハードウェアアクセラレーション構成のビルドも確認する。
- 動作確認は以下を含むこと。各テスト環境（NVIDIA GPU / Intel GPU / TURN サーバ等）は事前に確保しておくこと。
  - Sora への接続・映像送受信・切断
  - DataChannel の送受信
  - CUDA エンコード（NVIDIA GPU 搭載環境）
  - VPL エンコード（Intel GPU 搭載環境）
  - TURN / プロキシ経由の接続
  - 各プラットフォームでの正常終了確認
- 12 マイルストーン分の追従となるため、作業途中で行き詰まった場合の中間マイルストーンでの段階的アプローチ（例: M138 → M145 → M150）も検討すること。

### M150 の検証ケース

`M150` の runtime 変更は `HW` エンコーダの実機検証が主眼である。`--fake-capture-device` が使えるなら fake capture で十分で、実カメラが必要なのはデバイス列挙やフォーマットネゴシエーションを確認する場面だけに限る。

#### 1. `M150` 変更由来 - `E2E` 未カバー

- [X] `macOS arm64` で `Sora --data-channel-signaling true` で接続 -> 切断 -> 再接続する
  - 該当変更: `deadline_timer` -> `steady_timer`
  - 備考: `E2E` で `DCS` を使うテストが存在しない。

#### 2. 一般 canary - プラットフォーム別 `E2E` 未カバー

- [X] `macOS arm64`
  - 検証内容: 内蔵カメラ（MacBook）/ USB カメラ（Mac Studio 等）で映像送信 + 内蔵マイク音声
  - 理由: `E2E` は全テスト `--fake-capture-device` のみ。実カメラ / 音声デバイスのパスは通っていない。
- [X] `macOS arm64`
  - 検証内容: `--list-devices` でカメラ / 音声デバイスが正しく列挙されるか確認する
  - 理由: `E2E` で呼ぶテストがない。
- [X] `Ubuntu x86_64`
  - 検証内容: NVIDIA USB カメラ（V4L2）で映像送信 + USB マイク音声
  - 理由: 同上。
- [X] `Ubuntu x86_64`
  - 検証内容: VPL USB カメラ（V4L2）で映像送信 + USB マイク音声
  - 理由: 同上。
- [X] `Raspberry Pi`
  - 検証内容: USB カメラ（V4L2）での映像送信
  - 理由: `RPi` `E2E` は `libcamera`（CSI）のみ。`V4L2` 直接パスは未カバー。
- [X] `Raspberry Pi`
  - 検証内容: `--use-libcamera-native` + `H.264`（`Pi 4`）
  - 理由: `E2E` では `--use-libcamera-native` 未使用。
- [X] `Windows x86_64` / `NVIDIA Video Codec`
  - 検証内容: USB カメラ + 既定の音声入出力で映像送受信し、`H264` / `H265` / `AV1` の送信ができることを確認する
  - 理由: `E2E` matrix からコメントアウト中。
- [X] `Windows x86_64` / `Intel VPL`
  - 検証内容: USB カメラ + 既定の音声入出力で映像送信し、`H264` / `H265` / `AV1` / `VP9` の送信ができることを確認する
  - 理由: `E2E` matrix からコメントアウト中。
- [X] `Jetson`
  - 検証内容: `./momo --log-level 0 --hw-mjpeg-decoder true --h264-encoder jetson --video-input-device "<VIDEO_DEVICE>" --audio-input-device "<AUDIO_DEVICE>" sora --signaling-urls <SIGNALING_URL> --channel-id sora --role sendonly --video true --audio false --video-codec-type H264 --video-bit-rate 1000` を実行すると、`no matching device found` になり `failed to create capturer` で失敗した
  - 理由: ログに `no matching device found` と `failed to create capturer` が出てしまい、未対応のカメラデバイスでは実行できない
  - 対応: mjpeg に対応したデバイスで問題なく動作を確認できた
- [X] `Jetson`
  - 検証内容: `./momo --log-level 0 --resolution HD --hw-mjpeg-decoder true --h264-encoder jetson sora --signaling-urls <SIGNALING_URL> --channel-id sora --role sendonly --video true --audio false --video-codec-type H264 --video-bit-rate 1000` を実行し、`--list-devices` で確認できる `MJPG` 対応デバイスと対応解像度を使った配信ができた
  - 理由: `--list-devices` の結果と実行結果で確認できたため。

#### 3. 優先度

- 絶対やる（1 - 3）: `m150` 変更で壊れている可能性があり、`E2E` では絶対に検出できない。
- できればやる（4, 6, 8）: 各プラットフォームで実カメラ + 音声が最低 1 回通れば安心。`fake capture` とキャプチャパスが違うので `E2E` が通っていても安心できない。
- その他（`--list-devices`、`--hw-mjpeg-decoder`、`--use-libcamera-native`、`Windows` の `NVIDIA` / `VPL`、`Jetson`）:
  - オプション機能や特定環境の確認なので、使う人がいれば優先して確認する。

### 4. CHANGES.md とバージョン更新

- API 追従作業完了後、変更内容をまとめて `CHANGES.md` の `## develop` に `[UPDATE]` で追記する（コンパイルエラー修正の過程ではなく、結果としての変更内容を記録する）。
- CI 全通りの確認後、feature ブランチから develop へ PR を作成し、レビュー・マージする。
- マージ後、develop の HEAD で `VERSION`（canary）を繰り上げてタグを打ち、GitHub Release を発行する。例: `2025.2.0-canary.4` → `2025.3.0-canary.0`（または `2026.1.0-canary.0`）。年バージョンが変わる場合はチームで確認して決定すること。

## 完了条件

- `DEPS` の `WEBRTC_BUILD_VERSION` が `m150.7871.0.0` になっている。
- M140 を経由する段階的追従で、途中段階のビルド結果と差分を把握したうえで M150 に到達している。
- M150 の libwebrtc に対して全対象プラットフォーム（Windows / macOS / Ubuntu x86_64 / arm（Jetson）/ Raspberry Pi）のビルドが CI 上で成功している。一部プラットフォームのみ成功している状態での部分マージは行わず、全プラットフォーム成功を必須とする。
- 必要な API 追従が `src/`（`src/sora-cpp-sdk/` 含む）に反映され、`CHANGES.md` に記録されている。
- 動作確認（Sora 接続・映像送受信・DataChannel・HW エンコード）が各プラットフォームで完了している。
- `VERSION`（canary）が繰り上げられ、GitHub Release が公開されている。
- 特定プラットフォームのビルドが 1 週間以上通らない場合は、担当者が原因を分析し、段階的アプローチへの移行または子 issue への切り出しを判断すること。

## 解決方法

### M140 でやったこと

- `M138` からのジャンプアップはやめ、まず `M140` へ更新した。`13ce1700` で libwebrtc を `m140` に上げ、`b6923483` で `sora-cpp-sdk` の `LAST_UPDATED` を更新し、`fb23671d` で macOS の `clang` / `libc++` を `libwebrtc` 管理下のものへ切り替え、`11d32f3f` で `PeerConnectionFactory` の GCM 暗号スイート設定を `RTCConfiguration` 側へ移した。
- `M140` へ上げた際に出た macOS arm64 のリンクエラーを修正した。`Xcode` 付属の `clang` / `libc++` ではなく、`libwebrtc` 管理下の `clang` / `libc++` を使うようにし、`std::__1` と `std::__Cr` の ABI 不一致が起きないようにした。
- `run.py` と `CMakeLists.txt` をそろえ、`-nostdinc++`、`-isystem`、`_LIBCPP_ABI_NAMESPACE=Cr`、`_LIBCPP_ABI_VERSION=2` を使う構成にした。`Boost` を含む依存ライブラリも同じ `libc++` 系で再ビルドし、`std::__1::basic_string_view` を含む未解決シンボルが解消されることを、`nm` と実際のリンクログで確認した。

### M146 でやったこと

- `d1dea570` で `m146.7680.3.1` への更新を行い、`SSLCertificateVerifier` を `Verify` から `VerifyChain` に変更し、`PeerConnectionFactory` を `Environment` を受け取る 3 引数構成に追従し、`AudioDeviceBuffer` も `Environment` 受け取りに変更し、`hwenc_nvcodec` / `hwenc_v4l2` / `hwenc_vpl` の `OnEncodedImage` 失敗時の扱いを `error` 返却から `warning` ログ出力へ変えた。
- `shiguredo/momo#454` で `M146` の `PR` はマージ済みである。

### M148 でやったこと

- `d6826972` で `m148.7778.4.0` へ更新し、`Boost` を `1.91.0` に上げ、`CMake` を `4.3.2` に上げ、`VPL` を `v2.16.0` に上げた。
- macOS ビルドで `BOOST_ASIO_DISABLE_STD_ATOMIC_WAIT` を追加し、`webrtc-build` 同梱の `libc++` と組み合わせた `async_connect` のハングを回避した。
- `m148` のビルドが通った。

### M150 でやったこと

- `0943b6d5` で `WEBRTC_BUILD_VERSION` を `m150.7871.0.0` に上げた。
- `1afb5430` で `src/sora-cpp-sdk/` 配下を `m150` に合わせて更新した。
- `bd42e834` で `__FUNCTION__` を `__func__` に置き換え、`boost::system::error_code` のログ出力を `error.message()` や `ec.to_string()` に変えた。
- `f774f692` で `sora_data_channel_on_asio.h` の `deadline_timer` を `steady_timer` に変更した。
- `m150` への更新後、ビルドは通っている。
- 変更量が大きいため、ブランチを分割して PR を出す方針に切り替えた。
- `shiguredo/momo#455` で `M150` の `PR` はマージ済みである。

### ブランチ分割

PR で確認する内容が多くなってしまうため、分割して PR を発行した。

- `m140` から `m146` までを 1 本のブランチとして切り、そこから先は別ブランチに分けて `PR` を出す。
- `m146` の `PR` を先にマージし、その後に `m150` の `PR` をマージする。
