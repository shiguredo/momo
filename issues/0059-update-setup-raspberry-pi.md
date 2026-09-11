# SETUP_RASPBERRY_PI.md を現行の Momo に合わせて更新する

- Created: 2026-09-10
- Completed: {YYYY-MM-DD}
- Branch: feature/update-setup-raspberry-pi
- Polished: 2026-09-11
- Reporter: @torikizi

## 目的

`doc/SETUP_RASPBERRY_PI.md` は Raspberry Pi OS 向けのセットアップ手順と Raspberry Pi 固有のオプションを案内するドキュメントだが、Raspberry Pi 専用カメラ (CSI カメラ) のキャプチャーが libcamera に移行した後も旧カメラスタック (V4L2) を前提とした説明が残っている。何が libcamera で何が V4L2 なのかを整理し、現行の Momo の実装と一致する内容に更新する。

## 現状

- CSI カメラの手順 (「Raspberry Pi OS で Raspberry Pi 用カメラなどの CSI カメラを利用する場合」) は `--use-libcamera` を案内する形に更新済みで、raspi-config の Camera Enable と `modprobe bcm2835-v4l2` の手順は削除されている (コミット 80dd9fd6)
- 「Raspberry Pi 向けの追加のオプション」の `--force-i420` は Raspberry Pi 専用カメラ向けのオプションとして説明されている。実際には `src/util.cpp` で `MomoArgs::force_i420` に設定され、`src/main.cpp` で `sora::V4L2VideoCapturerConfig::force_i420` に渡るだけであり、libcamera キャプチャー (`src/sora-cpp-sdk/src/hwenc_v4l2/libcamera_capturer.cpp` の `LibcameraCapturer`) はこの値を参照しない。`--use-libcamera` 指定時には効果がない
- 「Raspberry Pi 専用カメラでパフォーマンスが出ない」の `--hw-mjpeg-decoder` も Raspberry Pi 専用カメラ向けとして案内されている。`--hw-mjpeg-decoder` は `MomoArgs::hw_mjpeg_decoder` から `src/main.cpp` で 2 系統に渡る。キャプチャー側は `sora::V4L2VideoCapturerConfig::use_native` になり、`USE_V4L2_ENCODER` かつ `--use-libcamera` 未指定のとき `sora::V4L2Capturer` の生成に使われる。`--use-libcamera` 指定時は `sora::LibcameraCapturer` を生成し、このキャプチャーは `use_native` を参照しない。libcamera のネイティブバッファ出力は `--use-libcamera-native` (`LibcameraCapturerConfig::native_frame_output`) が制御する。一方、同じ `args.hw_mjpeg_decoder` は capturer 種別に関係なく `RTCManagerConfig::hardware_encoder_only` にも入り、`MomoVideoEncoderFactory` がソフトウェアエンコーダーを拒否する。したがって `--use-libcamera` と `--hw-mjpeg-decoder true` を同時指定すると、MJPEG のハードウェアデコードや V4L2 リサイズは走らないが、ソフトウェアエンコーダー禁止の副作用はある。CSI カメラの性能案内に `--hw-mjpeg-decoder` を使うのは誤りであり、性能は `doc/LIBCAMERA.md` のとおり `--use-libcamera-native` を案内する
- 「Raspberry Pi 専用カメラが利用できない」という見出しの下に「Momo 2023.1.0 から利用できるようになりました」と `--use-libcamera` の説明があり、見出しと内容がかみ合っていない
- インストール手順は `sudo apt-get install libcamera0.6` とバージョンを固定している。リリースバイナリが依存する libcamera.so のバージョンはリリースごとに変わり、そのたびに不足パッケージの対応が必要になっている (`CHANGES.md` の libcamera 0.5 / 0.6 / 0.7 関連の修正)
- USB カメラ向けに案内している `/boot/firmware/config.txt` の `gpu_mem=256` / `force_turbo=1` / `avoid_warnings=2` は旧カメラスタック時代の設定であり、現行の Raspberry Pi OS で必要なのか確認されていない

## 設計方針

- `doc/SETUP_RASPBERRY_PI.md` を CSI カメラ (libcamera) と USB カメラ (V4L2) の 2 系統に整理する
  - CSI カメラは `--use-libcamera` / `--use-libcamera-native` を基本とし、詳細は `doc/LIBCAMERA.md` に集約する
  - USB カメラは `--hw-mjpeg-decoder` などの V4L2 向けオプションを案内する
- `--force-i420` は V4L2 キャプチャー向けであり、`--use-libcamera` 指定時には効果がないことを明記する。CSI カメラ (libcamera) のトラブルシューティングとしては案内しない
- `--hw-mjpeg-decoder` は USB カメラ (V4L2) の MJPEG ハードウェアデコード / リサイズ向けとして案内する。CSI カメラの性能改善としては案内しない。`--use-libcamera` と同時指定してもキャプチャー経路は変わらず、`hardware_encoder_only` によるソフトウェアエンコーダー禁止だけが残ることは、案内する場合に誤って「完全に無効」と書かない
- libcamera パッケージの案内は、GitHub Releases の Raspberry Pi OS 向けバイナリ (本ドキュメントがダウンロード対象にしているもの) を使う利用者向けとする。develop / canary は対象にしない
  - 不足ライブラリの確認手順の正は `ldd ./momo | grep not` とする。Raspberry Pi OS Lite 節だけに閉じず、パッケージ導入の基本手順にする
  - `libcamera0.6` のような SONAME 固定の `apt-get install` を唯一の手順としては残さない。リリースごとに `libcamera.so` の SONAME が変わるため、固定名はすぐに現行リリースとずれる
  - 例としてパッケージ名を書く場合は、対象リリースバイナリを `ldd` したときの未解決ライブラリに合わせた例であることを明記する。実装時に最新 GitHub Release の Raspberry Pi OS 向けバイナリで `ldd` し、その結果に合わせる
- `gpu_mem` / `force_turbo` / `avoid_warnings` は最新の Raspberry Pi OS を入れた実機で、USB カメラ + `--hw-mjpeg-decoder` の性能改善に今も必要かを確認する。不要なら削除する。必要なら USB カメラ (V4L2) 向けとして残し、CSI カメラや旧カメラスタックの前提としては書かない。確認できないうちは推測で削除しない (`shiguredo-doc` の、推測で文章を変えない方針)
- 見出しと本文の対応を見直し、「利用できない」という見出しの下に「利用できるようになった」と書くような構成を解消する

## 完了条件

- `doc/SETUP_RASPBERRY_PI.md` の記述が現行の Momo の実装と一致している
- CSI カメラ (libcamera) と USB カメラ (V4L2) それぞれで有効なオプションが正しく案内されている
- `--force-i420` が Raspberry Pi 専用カメラ (libcamera) 向けとして案内されていない
- `--hw-mjpeg-decoder` が CSI カメラの性能改善として案内されていない。USB カメラ向けに案内する場合、キャプチャー経路と `hardware_encoder_only` の効果を実装と矛盾させない
- 不足ライブラリの確認が `ldd ./momo | grep not` を正として書かれている。SONAME 固定の `libcamera0.6` だけを唯一のインストール手順として残していない。例示するパッケージ名がある場合は、最新 GitHub Release の Raspberry Pi OS 向けバイナリの `ldd` 結果と一致している
- `gpu_mem` などの設定について、実機確認の結果が反映されている (不要なら削除、必要なら USB カメラ向けとして残す)。確認前に推測で削除されていない
- `doc/LIBCAMERA.md` と内容が矛盾しない

## 解決方法

未着手 (PR 作成後に追記する)
