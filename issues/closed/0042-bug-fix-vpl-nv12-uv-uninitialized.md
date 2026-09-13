# VPL デコーダが NV12 の `Data.UV` を設定せず色面が壊れる

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-vpl-nv12-uv-uninitialized
- Polished: {YYYY-MM-DD}

## 目的

Intel VPL ハードウェアデコーダが NV12 出力を I420 に変換するとき、`mfxFrameSurface1` の `Data.UV` を設定していない。`libyuv::NV12ToI420` が未初期化または null の UV 面を読むため、色が壊れる・クラッシュする可能性がある。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_decoder.cpp` の `VplVideoDecoderImpl` がサーフェス確保時に `Data.Y` / `Data.U` / `Data.V` だけを設定する
- 同じファイルのデコード完了後に `libyuv::NV12ToI420` へ `out_surface->Data.UV` を渡す
- `src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_encoder.cpp` のサーフェス確保はエンコーダ入力向けに `Data.U` / `Data.V` を使っており、デコーダの NV12 読み出し経路とは別である
- Intel Media SDK / VPL の NV12 では UV 面は `Data.UV` (または `Data.U` と同一アドレス) を使う。`Data.UV` を置かないと変換先が不定になる

## 設計方針

- デコーダのサーフェス確保で `Data.UV` を UV プレーン先頭 (`Data.Y + width * height`) に設定する。`Data.U` と一致させる
- エンコーダ側の `Data.U` / `Data.V` 設定は本 issue の対象外とする
- 修正は momo の vendored コードと sora-cpp-sdk 本体の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- デコーダが確保する NV12 サーフェスで `Data.UV` が UV プレーンを指す
- NV12 デコード後の I420 変換で未初期化ポインタを渡さない
- 既存の VPL デコード経路が従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
