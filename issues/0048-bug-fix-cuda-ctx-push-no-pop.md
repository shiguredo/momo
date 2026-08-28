# CUDA の `cuCtxPushCurrent` が例外時に Pop されずコンテキストが壊れる

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-cuda-ctx-push-no-pop
- Polished: {YYYY-MM-DD}

## 目的

NvCodec CUDA デコーダの初期化が `cuCtxPushCurrent` のあと `cuvidGetDecoderCaps` を呼び、失敗時に例外を投げる。`cuCtxPopCurrent` がその後にあるため、例外パスでは Pop されず CUDA コンテキストスタックが壊れる。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp` の `NvCodecDecoderCuda` コンストラクタが `CUDA_DRVAPI_CALL(dyn::cuCtxPushCurrent(...))` のあと `NVDEC_API_CALL(dyn::cuvidGetDecoderCaps(&decodecaps))` し、成功時だけ `cuCtxPopCurrent` する
- `CUDA_DRVAPI_CALL` は失敗で `NVDECException` を throw する
- `NVDEC_API_CALL` も失敗で例外を投げる想定である
- Push 成功後に Caps 取得が失敗すると Pop に到達しない

## 設計方針

- Push / Pop を RAII にする。デストラクタで必ず `cuCtxPopCurrent` する。二重 Pop はしない
- 既存の成功パスの意味 (Push → Caps → Pop) は変えない
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- `cuvidGetDecoderCaps` が失敗しても `cuCtxPopCurrent` が走る
- 対応コーデックでは従来通り初期化できる
- Push 失敗時は Pop しない

## 解決方法

未着手 (PR 作成後に追記する)
