# Jetson デコーダが `CHUNK_SIZE` を超える入力を検証せず memcpy する

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-jetson-decoder-chunk-size-memcpy
- Polished: 2026-09-13

## 目的

Jetson ハードウェアデコーダが出力プレーンへ入力フレームを `memcpy` するとき、プレーン容量 (mmap 実長) と `input_image.size()` を比較しない。プレーン容量は `setOutputPlaneFormat(..., CHUNK_SIZE)` の要求値 (4000000) に基づく。容量を超えるアクセスユニットでプレーンを越境書き込みし、クラッシュ・メモリ破壊する。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_decoder.cpp` で `CHUNK_SIZE` を 4000000 と定義し、`setOutputPlaneFormat(..., CHUNK_SIZE)` (`IsSupported()` / `JetsonConfigure()`) に渡す
- 同じファイルの `Decode()` が `memcpy(buffer->planes[0].data, input_image.data(), input_image.size())` し、続けて `bytesused` に `input_image.size()` を入れる
- `input_image.size()` は外部由来で、プレーン容量との比較が無い (data の NULL チェックのみ)
- プレーン容量の正は `NvBufferPlane::length` (uint32_t) で、`NvV4l2ElementPlane::queryBuffer()` が `VIDIOC_QUERYBUF` で設定する mmap 実長である。要求値 `CHUNK_SIZE` とはドライバ次第で一致しない
- `0022` は V4L2 (`V4L2DecodeConverter::Decode`) の mmap 越境であり、比較対象を mmap 実長にする点は類似するが、本指摘は Jetson の `NvVideoDecoder` 出力プレーンである
- `hwenc_jetson` は momo 独自コードであり、sora-cpp-sdk 2026.2.1 には存在しない (0013 で確認済み)

## 設計方針

- `Decode()` のコピー前に `input_image.size()` が `buffer->planes[0].length` (mmap 実長) を超えていないか検証する。比較は `>` とする (等しければちょうど収まるため正常)
- 超過時は `memcpy` せず `RTC_LOG(LS_ERROR)` で英語のエラーログを出し、`WEBRTC_VIDEO_CODEC_ERROR` を返す
- バッファを `dqBuffer()` で取得した場合は、`qBuffer()` でキューへ戻してからエラーを返す。戻さないと `getNumQueuedBuffers()` が減ったままになり、次の `Decode()` が `getNthBuffer(getNumQueuedBuffers())` でキュー済みバッファを掴み、デコード前のデータを上書きする (リングの前提が崩れる)。`getNthBuffer()` 経由で取得したときは戻す必要は無い
- 比較対象を要求値 `CHUNK_SIZE` ではなく mmap 実長とするのは `0022` と同じ方針である
- momo のみ修正する。sora-cpp-sdk への反映は不要である

## 完了条件

- mmap 実長を超える入力で越境書き込みしない
- 通常サイズの入力は従来通りデコードされる
- 超過時にエラーログが出力される
- 超過エラー後も、以降の通常サイズ入力がデコードされ続ける (キューが崩れない)

## 解決方法

未着手 (PR 作成後に追記する)
