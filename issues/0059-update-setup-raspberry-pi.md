# SETUP_RASPBERRY_PI.md を現行の Momo に合わせて更新する

- Created: 2026-09-10
- Completed: {YYYY-MM-DD}
- Branch: feature/update-setup-raspberry-pi
- Polished: {YYYY-MM-DD}
- Reporter: @torikizi

## 目的

`doc/SETUP_RASPBERRY_PI.md` は Raspberry Pi OS 向けのセットアップ手順と Raspberry Pi 固有のオプションを案内するドキュメントだが、Raspberry Pi 専用カメラ (CSI カメラ) のキャプチャーが libcamera に移行した後も旧カメラスタック (V4L2) を前提とした説明が残っている。何が libcamera で何が V4L2 なのかを整理し、現行の Momo の実装と一致する内容に更新する。

## 現状

- CSI カメラの手順 (「Raspberry Pi OS で Raspberry Pi 用カメラなどの CSI カメラを利用する場合」) は `--use-libcamera` を案内する形に更新済みで、raspi-config の Camera Enable と `modprobe bcm2835-v4l2` の手順は削除されている (コミット 80dd9fd6)
- 「Raspberry Pi 向けの追加のオプション」の `--force-i420` は Raspberry Pi 専用カメラ向けのオプションとして説明されている。実際には `src/util.cpp` で `MomoArgs::force_i420` に設定され、`src/main.cpp` で `sora::V4L2VideoCapturerConfig::force_i420` に渡るだけであり、libcamera キャプチャー (`src/sora-cpp-sdk/src/hwenc_v4l2/libcamera_capturer.cpp` の `LibcameraCapturer`) はこの値を参照しない。`--use-libcamera` 指定時には効果がない
- 「Raspberry Pi 専用カメラでパフォーマンスが出ない」の `--hw-mjpeg-decoder` も Raspberry Pi 専用カメラ向けとして案内されている。`--hw-mjpeg-decoder` は `MomoArgs::hw_mjpeg_decoder` から `use_native` として V4L2 系キャプチャーの生成にのみ使われ、libcamera キャプチャーでは使われない。libcamera のネイティブバッファ出力は `--use-libcamera-native` が制御する
- 「Raspberry Pi 専用カメラが利用できない」という見出しの下に「Momo 2023.1.0 から利用できるようになりました」と `--use-libcamera` の説明があり、見出しと内容がかみ合っていない
- インストール手順は `sudo apt-get install libcamera0.6` とバージョンを固定している。リリースバイナリが依存する libcamera.so のバージョンはリリースごとに変わり、そのたびに不足パッケージの対応が必要になっている (`CHANGES.md` の libcamera 0.5 / 0.6 / 0.7 関連の修正)
- USB カメラ向けに案内している `/boot/firmware/config.txt` の `gpu_mem=256` / `force_turbo=1` / `avoid_warnings=2` は旧カメラスタック時代の設定であり、現行の Raspberry Pi OS で必要なのか確認されていない

## 設計方針

- `doc/SETUP_RASPBERRY_PI.md` を CSI カメラ (libcamera) と USB カメラ (V4L2) の 2 系統に整理する
  - CSI カメラは `--use-libcamera` / `--use-libcamera-native` を基本とし、詳細は `doc/LIBCAMERA.md` に集約する
  - USB カメラは `--hw-mjpeg-decoder` などの V4L2 向けオプションを案内する
- `--force-i420` / `--hw-mjpeg-decoder` は V4L2 キャプチャー向けであることを明記し、libcamera のトラブルシューティングとして案内しない
- libcamera パッケージの案内は、リリースバイナリが依存するバージョンと突き合わせられるようにする。`ldd ./momo | grep not` で不足パッケージを確認する手順を基本にし、`libcamera0.6` のような固定バージョンは現行バイナリに合わせて更新する
- `gpu_mem` などの旧カメラスタック前提の設定は、最新の Raspberry Pi OS を入れた実機で必要性を確認し、不要なら削除する
- 見出しと本文の対応を見直し、「利用できない」という見出しの下に「利用できるようになった」と書くような構成を解消する

## 完了条件

- `doc/SETUP_RASPBERRY_PI.md` の記述が現行の Momo の実装と一致している
- CSI カメラ (libcamera) と USB カメラ (V4L2) それぞれで有効なオプションが正しく案内されている
- libcamera では効果のない `--force-i420` / `--hw-mjpeg-decoder` が Raspberry Pi 専用カメラ向けとして案内されていない
- インストールする libcamera パッケージの案内が現行のリリースバイナリと整合している
- 旧カメラスタック前提の手順・設定が残っていない
- `doc/LIBCAMERA.md` と内容が矛盾しない

## 解決方法

未着手 (PR 作成後に追記する)
