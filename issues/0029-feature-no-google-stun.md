# --no-google-stun の P2P モード対応

Created: 2026-05-04
Model: Opus 4.7

## 概要

`--no-google-stun` が CLI でパースされるが P2P モードでは未使用。Ayame モードでは反映されているが P2P モードには渡されていないため、`--no-google-stun` を指定しても常に `stun:stun.l.google.com:19302` が追加されてしまう。

## 根拠

- `src/main.rs:48` で `_no_google_stun` (アンダースコア付き) に束縛されている
- `MomoConfig.no_google_stun` は `#[cfg(feature = "ayame")]` で gate されており、Ayame モードでのみ伝搬される
- `src/p2p/webrtc.rs:364` 付近で `IceServer::new()` に対して常に `stun:stun.l.google.com:19302` を追加している
- Sora モードは sora_sdk が ICE サーバーを内部で管理しているため、本オプションの対象外

## 必要な実装

- `MomoConfig.no_google_stun` の `#[cfg(feature = "ayame")]` を外し、共通フラグにする
- `main.rs` の `_no_google_stun` を `no_google_stun` にリネーム (実利用するためアンダースコアを外す)
- `P2PConfig` に `pub no_google_stun: bool` を追加し、`run_p2p` で `momo_config.no_google_stun` を渡す
- `src/p2p/webrtc.rs` で `--no-google-stun` 指定時は Google STUN を追加しないようにする

## 参考

- momo の `--no-google-stun` 実装: 全モードで Google STUN サーバー追加をスキップ
