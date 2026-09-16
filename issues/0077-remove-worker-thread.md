# libwebrtc の worker_thread 削除に追随する

- Created: 2026-09-15
- Completed: {YYYY-MM-DD}
- Branch: feature/remove-worker-thread
- Polished: {YYYY-MM-DD}

## 目的

libwebrtc の issue 558821261「Deprecate and remove PeerConnectionFactoryDependencies::worker_thread」の削除系 CL がマージされると、`PeerConnectionFactoryDependencies::worker_thread` と `PeerConnectionFactoryInterface::worker_thread()` が無くなる。

削除系の CL は 499302 / 501640 / 501720 / 502000 / 502500 / 502860 / 502940 / 502960 である。momo に残っている worker thread の参照を削除し、削除後の libwebrtc でビルドできるようにする。

## 現状

- 0076-refactor-unify-worker-thread で専用の worker thread は廃止済みだが、`src/rtc/rtc_manager.cpp` の `RTCManager::RTCManager` に次の 2 箇所が残る
  - `dependencies.worker_thread = network_thread_.get()` による network thread の設定
  - `webrtc::PeerConnectionFactoryProxy::Create(factory->signaling_thread(), factory->worker_thread(), factory)` の第 2 引数の `factory->worker_thread()`
- momo の libwebrtc 依存は webrtc-build の直接参照であり、`DEPS` の `WEBRTC_BUILD_VERSION=m150.7871.3.0` で固定している
- 前提条件は webrtc-build が 558821261 の削除系 CL を含むバージョンをリリースしていることである。現時点では存在しないため、本 issue には着手できない
- `webrtc::PeerConnectionFactory::network_thread()` は private であり、factory から network thread を取り出すことができない

## 設計方針

- `DEPS` の `WEBRTC_BUILD_VERSION` を 558821261 の削除系 CL を含むバージョンに更新する
- `dependencies.worker_thread` の行を削除する。CL 501620 により worker thread を設定しない場合は network thread が使われる
- `webrtc::PeerConnectionFactoryProxy::Create` の第 2 引数を network thread に変更する。`webrtc::PeerConnectionFactory::network_thread()` は private のため、`RTCManager` 自身の `network_thread_` を使う
- 挙動変更・機能追加・バグ修正は行わない

## 完了条件

- `WEBRTC_BUILD_VERSION` が 558821261 の削除系 CL を含むバージョンに更新されている
- `git grep worker_thread` が issues 以外で 0 件になる
- Windows / macOS / Ubuntu / Raspberry Pi OS / Jetson でビルドが通る
- Sora / Ayame / P2P モードで音声と映像の送受信が従来どおり動作する
- macOS / Linux で `--audio-input-device` / `--audio-output-device` を指定したときに音声デバイスが従来どおり再適用される
- `CHANGES.md` の `## develop` に追記する

## 解決方法

{YYYY-MM-DD} に追記する
