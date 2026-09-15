# RTCManager の worker thread を network thread に統一する

- Created: 2026-09-15
- Completed: 2026-09-15
- Branch: feature/refactor-unify-worker-thread
- Polished: {YYYY-MM-DD}

## 目的

libwebrtc の issue 558821261「Deprecate and remove PeerConnectionFactoryDependencies::worker_thread」で worker thread が廃止される。`PeerConnectionFactoryDependencies::worker_thread` と `PeerConnectionFactoryInterface::worker_thread()` は将来削除される。

CL 501620「Default worker thread to network thread」と CL 502480「Warn when a distinct worker thread is configured」はマージ済みであり、削除系の CL (499302 / 501640 / 501720 / 502000 / 502500 / 502860 / 502940 / 502960) はレビュー中である。

momo は `RTCManager` で専用の worker thread を生成して libwebrtc に渡している。専用の worker thread をやめて network thread を使うようにし、worker thread の削除に備える。

worker thread の利用箇所と API を全て無くす対応は、558821261 を実装した libwebrtc をマージした後に 0077-remove-worker-thread で行う。

## 現状

- worker thread の実装は `src/rtc/rtc_manager.cpp` / `src/rtc/rtc_manager.h` の `RTCManager` に閉じている
- `RTCManager::RTCManager` のコンストラクタで次の 3 つのスレッドを生成して `Start()` し、`dependencies` に設定している
  - `network_thread_`: `webrtc::Thread::CreateWithSocketServer()` で生成し、`dependencies.network_thread` に設定する
  - `worker_thread_`: `webrtc::Thread::Create()` で生成し、`dependencies.worker_thread` に設定する
  - `signaling_thread_`: `webrtc::Thread::Create()` で生成し、`dependencies.signaling_thread` に設定する
- `worker_thread_` は次の 5 箇所で使っている
  - `dependencies.adm = worker_thread_->BlockingCall(...)` による AudioDeviceModule の生成
  - `worker_thread_->BlockingCall(...)` による音声デバイスの再適用 (`SetAudioDevice`)
  - `webrtc::PeerConnectionFactoryProxy::Create(factory->signaling_thread(), factory->worker_thread(), factory)` の第 2 引数
  - `webrtc::VideoTrackSourceProxy::Create(signaling_thread_.get(), worker_thread_.get(), video_track_source)` の第 2 引数
  - `RTCManager::~RTCManager` の `worker_thread_->Stop()`
- `network_thread_` の利用は `dependencies` への設定、`Stop()`、メンバ宣言のみである
- AudioDeviceModule の生成と音声デバイスの再適用はいずれも接続確立前の `RTCManager` コンストラクタ内で実行される (`--audio-input-device` / `--audio-output-device` の再適用は `#if defined(__APPLE__) || defined(__linux__)` の中にある)

## 設計方針

- `worker_thread_` の生成 (`webrtc::Thread::Create()`) / `Start()` / `Stop()` と `src/rtc/rtc_manager.h` のメンバ宣言を削除し、factory の worker thread として network thread を使う
- `dependencies.worker_thread` には network thread を渡す。`dependencies.worker_thread` を未設定にすると現行の libwebrtc では PeerConnectionFactory が内部で専用スレッドを生成するため、明示的に network thread を設定する
- `dependencies.adm` の生成と音声デバイスの再適用の `BlockingCall` を network thread に移す
- `webrtc::PeerConnectionFactoryProxy::Create` と `webrtc::VideoTrackSourceProxy::Create` の第 2 引数を network thread に変更する
- `dependencies.worker_thread` の行と `webrtc::PeerConnectionFactoryProxy::Create` の `factory->worker_thread()` は、558821261 を実装した libwebrtc をマージした後に 0077-remove-worker-thread で削除する
- 挙動変更・機能追加・バグ修正は行わない

## 注意点

- 変更後は AudioDeviceModule の生成と音声デバイスの再適用が network thread の `BlockingCall` になる。network thread 上から network thread を `BlockingCall` する経路を作ると自己デッドロックするため、今後 network thread 上の処理から `BlockingCall` を追加しないこと

## 完了条件

- `worker_thread_` の生成 / `Start()` / `Stop()` とメンバ宣言が無くなる
- `git grep worker_thread` の結果が、`dependencies.worker_thread` に network thread を設定する行と `webrtc::PeerConnectionFactoryProxy::Create` の `factory->worker_thread()` の 2 箇所のみになる (どちらも 0077-remove-worker-thread で削除する)
- Windows / macOS / Ubuntu / Raspberry Pi OS / Jetson でビルドが通る
- Sora / Ayame / P2P モードで音声と映像の送受信が従来どおり動作する
- macOS / Linux で `--audio-input-device` / `--audio-output-device` を指定したときに音声デバイスが従来どおり再適用される
- `CHANGES.md` の `## develop` に追記する

## 解決方法

RTCManager の worker thread として network thread を使うようにした。

- `src/rtc/rtc_manager.cpp` から worker thread の生成、`Start()`、`Stop()`、`dependencies.worker_thread` への専用 thread の設定を削除し、network thread を渡すようにした
- ADM の生成と音声デバイスの再適用の `BlockingCall` を network thread に移し、`VideoTrackSourceProxy::Create` の第 2 引数も network thread に変更した
- `src/rtc/rtc_manager.h` から `worker_thread_` メンバを削除した
- `CHANGES.md` の `## develop` に追記した

確認:

- `python3 run.py build --package ubuntu-24.04_x86_64` が通ることを確認した
