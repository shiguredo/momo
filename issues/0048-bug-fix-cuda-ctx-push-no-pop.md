# CUDA の `cuCtxPushCurrent` が例外時に Pop されずコンテキストが壊れる

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-cuda-ctx-push-no-pop
- Polished: 2026-09-13

## 目的

NvCodec CUDA デコーダの初期化で `cuCtxPushCurrent` のあと `cuvidGetDecoderCaps` を呼ぶ。この `NVDEC_API_CALL` が失敗時に例外を投げると、後続の `cuCtxPopCurrent` に到達せず、構築したスレッドの CUDA コンテキストスタックに Push したままのエントリが残る。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp` の `NvCodecDecoderCuda` コンストラクタが `CUDA_DRVAPI_CALL(dyn::cuCtxPushCurrent(...))` のあと `NVDEC_API_CALL(dyn::cuvidGetDecoderCaps(&decodecaps))` し、成功時だけ `cuCtxPopCurrent` する
- `CUDA_DRVAPI_CALL` は失敗で `NVDECException` を throw する
- `NVDEC_API_CALL` も失敗で `NVDECException` を throw する
- Push 成功後に Caps 取得が失敗すると Pop に到達しない。try / catch や RAII は存在しない
- `NvCodecVideoDecoder::IsSupported()` (`src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_decoder.cpp`) は `NvCodecDecoderCuda` を構築して判定し、コンストラクタの例外を `catch (...)` で吸収して false を返す。Caps 取得が失敗した場合、例外はここで吸収されるが Push の残留は残る

## 設計方針

- Push / Pop をコンストラクタ本体のローカルスコープ RAII ガードにする。ガードが Caps 取得の直前で Push し、スコープ終了時 (成功・例外を問わない) に必ず `cuCtxPopCurrent` する。`NvCodecDecoderCuda` のメンバにはしない (メンバにすると成功パスの Pop がデコーダ破棄まで遅れ、既存の意味と変わる)
- ガードのデストラクタでは `cuCtxPopCurrent` の失敗を throw しない (ログのみにする)。例外展開中にデストラクタから throw すると `std::terminate` になるため、本 issue の目的である例外パスでより深刻な問題に変わる
- 二重 Pop はしない。既存の成功時の明示的な `cuCtxPopCurrent` 呼び出しを削除し、Pop はガードに一本化する
- 既存の成功パスの意味 (Push → Caps → Pop) は変えない
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- `cuvidGetDecoderCaps` が失敗しても `cuCtxPopCurrent` が走る
- 対応コーデックでは従来通り初期化できる
- 非対応コーデックの判定 (`bIsSupported == 0` で例外) と `IsSupported()` の false 判定は従来通り
- Push 失敗時は Pop しない

## 解決方法

未着手 (PR 作成後に追記する)
