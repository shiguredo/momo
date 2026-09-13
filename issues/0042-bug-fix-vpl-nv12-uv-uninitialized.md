# VPL デコーダが NV12 の `Data.UV` を設定せず色面が壊れる

- Created: 2026-08-28
- Completed: 2026-09-13
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

polish-issue による照合の結果、報告されたバグは現行実装では存在しない (実測ではなく、oneVPL の一次資料と Intel 公式サンプルとのソース照合で否定)。そのため closed とする。`Polished:` は更新しない。

### 照合結果

- `Data.UV` が未設定という前提は誤り。`src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_decoder.cpp` の `VplVideoDecoderImpl::InitVpl()` はサーフェス確保時に `surface.Data.U` を UV プレーン先頭 (`surface_buffer_.data() + i * size + width * height`) に設定しており、`Decode()` の `libyuv::NV12ToI420` に渡す `out_surface->Data.UV` はこの `Data.U` と常に同じ値を返す
- 根拠は `mfxFrameData` の構造。momo が使用する VPL は `DEPS` の `VPL_VERSION=v2.16.0` で、`buildbase.py` の `install_vpl()` が `https://github.com/intel/libvpl.git` のタグ `v2.16.0` を clone する。その `api/vpl/mfxstructures.h` (356〜368 行付近) では色面ポインタが Y 用・U 用・V 用の 3 つの無名 union になっており、`Data.UV` と `Data.U` は同一ストレージを共有する (`//!< UV channel for UV merged formats.` と `//!< U channel.` が同じ union メンバ)。したがって `Data.U` を設定した時点で `Data.UV` も同じ非 null アドレスを指す
- Intel 公式のデコードサンプル (`intel/libvpl` の `examples/api1x_core/legacy-decode/src/util.hpp` の `AllocateExternalSystemMemorySurfacePool()`、NV12 ケース) もサーフェスには `Data.Y` / `Data.U` / `Data.V` / `Data.Pitch` のみを設定し、`Data.UV` は設定しない。momo の実装はこの公式パターンと同一であり、`NV12ToI420` が未初期化または null の UV 面を読むことはない
- 本 issue の設計方針 (サーフェス確保で `Data.UV` を `Data.Y + width * height` に設定する) は、`Data.U` と同じストレージに同じ値を書き込むだけの no-op となり、挙動を変えない。完了条件 3 項目も現行実装で既に満たされている
- `issues/closed/0013-update-sora-cpp-sdk-sdl3-cli11.md` の finding「VPL `Data.UV` 未設定」は本 issue が引き継いだものであり、重複起票ではない。同 finding の「未解消」判定はソース照合によるもので、本照合で実装上の問題がないことを確認した
