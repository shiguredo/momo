# V4L2 変換器が `/dev/video10` 系のデバイスパスを直書きしている

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-v4l2-hardcoded-device-path
- Polished: 2026-09-13

## 目的

Raspberry Pi 向け V4L2 M2M 変換器がエンコーダ / スケーラ / デコーダのデバイスノードを `/dev/video11` / `/dev/video12` / `/dev/video10` に直書きしている。ノード番号が違うボードやカーネルでは open に失敗し、ハードウェアコーデックが使えない。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_converter.cpp` の `V4L2H264EncodeConverter::Init()` が `"/dev/video11"` を `open` する
- 同じファイルの `V4L2ScaleConverter::Init()` が `"/dev/video12"` を `open` する
- 同じファイルの `V4L2DecodeConverter::Init()` が `"/dev/video10"` を `open` する
- 機能 (H.264 encode / ISP scale / H.264 decode) による探索や、呼び出し元からのパス指定が無い

## 設計方針

- デバイスは capabilities (`V4L2_CAP_VIDEO_M2M_MPLANE` 等) と pixelformat で列挙して選ぶ。番号決め打ちをやめる
- 変換器ごとの選別規準は入出力方向と pixelformat の組で判定する
  - エンコーダ: OUTPUT が YUV420、CAPTURE が H264
  - デコーダ: OUTPUT が H264 または MJPEG、CAPTURE が YUV420
  - スケーラ: OUTPUT と CAPTURE が YUV420
  - エンコーダとデコーダはどちらも H264 を扱うため、pixelformat だけでは区別できず、入出力方向の判定が必須である
- 列挙に成功したが該当する M2M デバイスが見つからない場合、または列挙自体に失敗した場合は、現行パスをフォールバックにしてもよい。フォールバックする場合はログに使うパスを出す
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- `/dev/video10`〜`12` 以外のノードでも、対応する M2M デバイスがあれば open できる
- Raspberry Pi の現行配置でも従来通り動作する
- open 失敗時に使ったパスがログに残る

## 解決方法

未着手 (PR 作成後に追記する)
