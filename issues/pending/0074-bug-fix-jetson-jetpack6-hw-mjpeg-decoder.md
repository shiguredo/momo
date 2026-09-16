# Jetson / JetPack 6 で --hw-mjpeg-decoder が有効だと H.264 が送信できない

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-jetson-jetpack6-hw-mjpeg-decoder
- Polished: {YYYY-MM-DD}
- Reporter: @torikizi

## 目的

JetPack 6 の Jetson で、デフォルト有効の `--hw-mjpeg-decoder` を指定したまま H.264 を送信すると、受信側で映像が受信できない。`--hw-mjpeg-decoder=false` にすると受信できる。デフォルトのままでも送信できるようにする。

## 現状

- Jetson AGX Orin (JetPack 6.0 / L4T r36.3 / Ubuntu 22.04.5 / kernel 5.15.136-tegra) で `./momo --log-level 0 --no-audio-device --resolution HD --framerate 30 sora ... --role sendonly` を実行すると、受信側で映像が受信できない
- `--hw-mjpeg-decoder=false` を指定すると受信できる
- test / ayame / sora の各モードで確認している
- Jetson では `--hw-mjpeg-decoder` のデフォルトが `true` で、`MomoArgs::hw_mjpeg_decoder` から `V4L2VideoCapturerConfig::use_native` と `rtcm_config.hardware_encoder_only` に渡る
- native 時は `src/main.cpp` の分岐で `sora::NvCodecV4L2Capturer` または Jetson の capturer を使い、`src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_v4l2_capturer.cpp` / `src/sora-cpp-sdk/src/hwenc_jetson/jetson_v4l2_capturer.cpp` の native 経路を通る
- momo 2024.1.1 でも再現し、`doc/SETUP_JETSON.md` の既知の問題として `--hw-mjpeg-decoder=false` を案内している。同じ事象は公開 issue #355 にも記録されている
- JetPack 6.1 / 6.2.1 での解消有無は未確認 (0073 で JetPack を更新する)

## 設計方針

- JetPack 6 の native MJPEG デコード経路 (`NvCodecV4L2Capturer` / `JetsonV4L2Capturer`) が H.264 を送信できない原因を特定する
- JetPack 6.1 / 6.2.1 の L4T で解消しているかを確認する。解消している場合は 0073 (JetPack の更新) で取り込む
- 解消していない場合は native 経路の修正を momo の vendored コードと sora-cpp-sdk の両方に対して行う
- 修正が難しい場合は `--hw-mjpeg-decoder` の JetPack 6 でのデフォルトやフォールバックの扱いを見直す
- `doc/SETUP_JETSON.md` の既知の問題の記載を結果に合わせて更新する

## 完了条件

- JetPack 6 の Jetson で `--hw-mjpeg-decoder` を有効にしたまま H.264 を送信でき、受信側で映像が受信できる
- test / ayame / sora の各モードで確認できる
- `doc/SETUP_JETSON.md` の既知の問題の記載が解消状況に合わせて更新されている
- 修正が sora-cpp-sdk 側にも反映され、`LAST_UPDATED` が更新されている

## 解決方法

未着手 (原因の特定後に追記する)

## pending にした理由

JetPack 6 の native MJPEG デコード経路が原因で、JetPack SDK 側の更新 (0073) で解消する可能性がある。JetPack 6.0 では `--hw-mjpeg-decoder=false` の回避策があり、JetPack の更新待ちのため pending とする。
