# Raspberry Pi Camera で --list-devices に fps が表示されない問題を修正する

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-raspberry-pi-camera-list-devices-fps
- Polished: {YYYY-MM-DD}
- Reporter: @torikizi

## 目的

Raspberry Pi の Raspberry Pi Camera で `--list-devices` を実行したときに解像度と fps が表示されない。他の V4L2 デバイスと同様に、対応している解像度と fps が表示されるようにする。

## 現状

- `--list-devices` は `src/main.cpp` の `ListDevices` が `sora::EnumV4L2CaptureDevices` と `sora::FormatV4L2Devices` を呼ぶ
- `src/sora-cpp-sdk/src/v4l2/v4l2_device.cpp` の `EnumV4L2CaptureDevices` は `VIDIOC_ENUM_FRAMESIZES` の結果が `V4L2_FRMSIZE_TYPE_DISCRETE` の場合のみ採用し、それ以外は skip する。`VIDIOC_ENUM_FRAMEINTERVALS` も `V4L2_FRMIVAL_TYPE_DISCRETE` のみ採用する
- Raspberry Pi Camera の `/dev/video0` (unicam) や `/dev/video14` などは `V4L2_FRMSIZE_TYPE_DISCRETE` を返さない、または `VIDIOC_ENUM_FRAMESIZES` が失敗するため `format_desc.frame_sizes` が空になり、`FormatV4L2FormatDescription` が解像度と fps を表示しない (2025.1.0-canary.15 で確認)
- `feature/non-discrete` ブランチで `V4L2_FRMSIZE_TYPE_STEPWISE` / `V4L2_FRMSIZE_TYPE_CONTINUOUS` の列挙と、`VIDIOC_TRY_FMT` / `VIDIOC_G_PARM` によるフォールバックを試したが、フォールバックもエラーになり fps を取得できなかった
- 上流の sora-cpp-sdk の `src/v4l2/v4l2_device.cpp` も `DISCRETE` のみ対応で、momo の vendored コードと同じ
- `src/sora-cpp-sdk/` は sora-cpp-sdk から取り込んだ vendored コードで、修正は sora-cpp-sdk 側にも反映する必要がある (`src/sora-cpp-sdk/README.md` の `copy-from-sora-cpp-sdk.sh` / `copy-to-sora-cpp-sdk.sh`)

## 設計方針

- `V4L2_FRMSIZE_TYPE_CONTINUOUS` / `V4L2_FRMSIZE_TYPE_STEPWISE` と `V4L2_FRMIVAL_TYPE_CONTINUOUS` / `V4L2_FRMIVAL_TYPE_STEPWISE` を扱い、範囲とステップを表示できるようにする
- Raspberry Pi Camera のように `VIDIOC_ENUM_FRAMESIZES` / `VIDIOC_ENUM_FRAMEINTERVALS` が期待した結果を返さないドライバ向けに、`VIDIOC_G_FMT` / `VIDIOC_TRY_FMT` / `VIDIOC_G_PARM` を使った代替の取得を検討する
- `feature/non-discrete` のフォールバックが失敗した原因を調べ、ドライバが実際に何を返しているかをログで確認する
- 情報が取得できない場合は、誤った値を表示せず取得できないことが分かる表示にする
- 修正は momo の vendored コードと sora-cpp-sdk の両方に反映し、`LAST_UPDATED` を更新する

## 完了条件

- Raspberry Pi Camera で `--list-devices` に解像度と fps (取得できる場合) が表示される
- 他の V4L2 デバイスで従来どおり解像度と fps が表示される
- 情報を取得できないドライバでもクラッシュせず、取得できないことが分かる
- 修正が sora-cpp-sdk にも反映され、`LAST_UPDATED` が更新されている

## 解決方法

{YYYY-MM-DD} に追記する
