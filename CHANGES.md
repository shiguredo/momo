# 変更履歴

- CHANGE
    - 下位互換のない変更
- UPDATE
    - 下位互換がある変更
- ADD
    - 下位互換がある追加
- FIX
    - バグ修正

## develop

- [UPDATE] `libwebrtc` を `M90.4430@{#3}` に上げる
    - @voluntas
- [UPDATE] Boost 1.76.0 に上げる
    - @voluntas

## 2021.2.3

- [UPDATE] cmake を 3.20.0 に上げる
    - @melpon @voluntas
- [FIX] Jetson で HW エンコーダー指定時に、初期化タイミングによって、まれにセグフォが発生する問題を修正する
    - @enm10k

## 2021.2.2

- [UPDATE] cmake を 3.19.6 に上げる
    - @voluntas
- [UPDATE] `libwebrtc` を `M89.4389@{#7}` に上げる
    - @voluntas
- [FIX] `momo --verson` 実行時にエラーメッセージが出るのを修正
    - HW エンコーダが利用できるかをチェックしている際に利用できない場合に標準出力にエラーが出てしまうのを抑制するという方法で修正
    - @melpon @torikizi
- [FIX] OpenSSLCertificate では無くなったので BoringSSLCertificate を利用するように修正
    - TURN-TLS でセグフォする問題を解決
    - @melpon @tnoho

## 2021.2.1

- [FIX] ubuntu-18.04_armv8 向け libwebrtc ビルドで Jetson が動かない問題を解消する
    - @tnoho
- [UPDATE] `libwebrtc` を `M89.4389@{#5}` に上げる
    - @tnoho
- [UPDATE] cmake を 3.19.5 に上げる
    - @voluntas

## 2021.2

- [CHANGE] M89 で使用不可になったため対応ピクセルフォーマットから `NV12` を削除
    - @tnoho
- [UPDATE] `libwebrtc` を `M89.4389@{#4}` に上げる
    - @tnoho
- [UPDATE] JetPack を 4.5 にする
    - @tnoho
- [UPDATE] cmake を 3.19.4 に上げる
    - @voluntas

## 2021.1

- [UPDATE] cmake を 3.19.3 に上げる
    - @voluntas
- [UPDATE] GitHub Actions の macOS ビルドを macos-11.0 に変更する
    - @voluntas
- [UPDATE] Boost 1.75.0 に上げる
    - @voluntas
- [UPDATE] nlohmann/json を Boost.JSON に変更
    - @melpon
- [ADD] サイマルキャストの active と adaptivePtime に対応
    - @melpon
- [ADD] Apple Silicon 対応
    - @hakobera

## 2020.11

- [UPDATE] cmake を 3.19.2 に上げる
    - @voluntas
- [UPDATE] `libwebrtc` を `M88.4324@{#3}` に上げる
    - @voluntas
- [ADD] 統計情報を HTTP API で取得できるようにする
    - 統計情報サーバーのポート番号を指定する `--metrics-port INT` を追加
    - ループバック (127.0.0.1 で listen) がデフォルト、グローバル (0.0.0.0 で listen) アクセスを許可する場合は `--metrics-allow-external-ip` 引数を指定する
    - @hakobera

## 2020.10

- [CHANGE] `--use-native` を `--hw-mjpeg-decoder=<bool>` に変更して、ソフトウェアエンコーダとの組み合わせを不可にする
    - @melpon @tnoho
- [UPDATE] `libwebrtc` を `M88.4324@{#0}` に上げる
    - @tnoho @melpon @voluntas
- [UPDATE] cmake を 3.18.4 に上げる
    - @voluntas
- [ADD] Jetson Nano で VP8 HWA が利用できるようにする
    - @tnoho

## 2020.9

- [CHANGE] `ubuntu-16.04_x86_64_ros` を削除
    - @melpon
- [CHANGE] Jetson のフレーム変換順序を変更
    - @tnoho
- [CHANGE] `raspberry-pi-os_armv8` から SDL を削除
    - @melpon
- [CHANGE] `--multistream` と `--simulcast` に引数 true/false の指定を必要にする
    - @melpon
- [CHANGE] `--audio-bitrate` を `--audio-bit-rate` に変更
    - @melpon
- [CHANGE] `--video-bitrate` を `--video-bit-rate` に変更
    - @melpon
- [CHANGE] `--audio-codec` を `--audio-codec-type` に変更
    - @melpon
- [CHANGE] `--video-codec` を `--video-codec-type` に変更
    - @melpon
- [CHANGE] `--spotlight INT` を `--spotlight BOOL` に変更（true/false で指定）
    - @melpon
- [UPDATE] Boost 1.74.0 に上げる
    - @voluntas
- [UPDATE] cmake を 3.18.3 に上げる
    - @voluntas
- [UPDATE] json を 3.9.1 に上げる
    - @voluntas
- [UPDATE] `libwebrtc` を `M86.4240@{#10}` に上げる
    - @voluntas
- [ADD] `--spotlight-number INT` の引数を追加
    - @melpon
- [FIX] `SDL_PollEvent` の扱い方を修正
    - @melpon
- [FIX] スクリーンキャプチャの前に `CoInitializeEx` が必要になっていたのを修正
    - @torikizi @melpon

## 2020.8.1

- [UPDATE] `libwebrtc` を `M85.4183@{#2}` に上げる
    - @voluntas

## 2020.8

- [CHANGE] パッケージ名 `ubuntu-18.04_armv8_jetson` を `ubuntu-18.04_armv8_jetson_nano` と `ubuntu-18.04_armv8_jetson_xavier` に変更
    - @tnoho
- [ADD] macOS でも全画面スクリーンキャプチャ機能を利用できるようにする
    - @hakobera
- [ADD] Jetson Xavier シリーズで VP9 HWA を有効にする
    - @tnoho @melpon
- [ADD] サイマルキャストへの対応を追加
    - Sora モードで利用可能
    - @melpon @shino
- [UPDATE] Jetson の RootFS 構築方法をリポジトリからの取得に変更
    - @tnoho
- [UPDATE] `libwebrtc` を `M85.4183@{#1}` に上げる
    - @hakobera @voluntas
- [UPDATE] CLI11 を v1.9.1 にアップデートする
    - @voluntas
- [UPDATE] json を 3.8.0 に上げる
    - @voluntas
- [UPDATE] cmake を 3.18.0 に上げる
    - @voluntas

## 2020.7

- [UPDATE] `libwebrtc` を `M84.4147@{#7}` に上げる
    - @voluntas @melpon
- [UPDATE] cmake を 3.17.3 に上げる
    - @voluntas
- [UPDATE] Boost 1.73.0 にアップデートする
    - @voluntas
- [UPDATE] Jetson Nano 用のライブラリを NVIDIA L4T 32.4.2 に上げる
    - @melpon
- [ADD] Ubuntu 20.04 x86_64 に対応する
    - @hakobera
- [ADD] ビデオエンコーダデコーダを表示する `--video-codec-engines` を追加
    - @melpon
- [ADD] GitHub Actions の Boost をキャッシュ化する
    - @melpon
- [ADD] 全画面スクリーンキャプチャ機能を Windows / Linux 向けに追加する
    - ``--screen-capture`` 指定することで利用可能
    - @melpon
- [ADD] `raspberry-pi-os_armv8` を追加
    - @melpon
- [ADD] ビデオコーデックのエンジン名を指定できる機能を実装
    - @melpon
- [CHANGE] パッケージ名 `ubuntu-18.04_armv8_jetson_nano` を `ubuntu-18.04_armv8_jetson` に変更
    - @melpon
- [CHANGE] パッケージ名 `raspbian-buster_armv6` と `raspbian-buster_armv7` を `raspberry-pi-os_armv6` と `raspberry-pi-os_armv7` に変更
    - @melpon
- [FIX] Windows の ADM に専用の関数を使うようにする
    - @torikizi @melpon
- [FIX] build.sh の --no-tty オプションのヘルプメッセージの修正
    - @hakobera

## 2020.6

- [UPDATE] `libwebrtc` を `M84.4127@{#0}` に上げる
    - @voluntas
- [ADD] test モードの Momo と Ayame モードの Momo の相互接続を可能とする
    - @tnoho
- [CHANGE] ubuntu-16.04_armv7_ros ビルドを削除
    - @melpon

## 2020.5.2

- [FIX] AV1 が利用できなかったのを修正する
    - @torikizi @voluntas
- [UPDATE] `libwebrtc` を `M84.4104@{#0}` に上げる
    - @voluntas

## 2020.5.1

- [FIX] CMakeLists.txt のタイポを修正する
    - @azamiya @torikizi @tnoho @melpon

## 2020.5

リリース日: 2020.04.14

- [UPDATE] `libwebrtc` を `M83.4103@{#2}` に上げる
    - @voluntas
- [UPDATE] `libwebrtc` を `M81.4044@{#13}` に上げる
    - @voluntas
- [UPDATE] `cmake` を `3.17.1` に上げる
    - @voluntas
- [ADD] 実験的に AV1 に対応する
    - Sora モードでのみ利用可能
    - @voluntas @tnoho
- [FIX] Jetson Nano では ALSA ではなく PulseAudio を利用する
    - Jetson Nano でつながらない問題が発生するのを修正
    - @azamiya @torikizi @tnoho @melpon

## 2020.4

リリース日: 2020.04.01

- [UPDATE] `libwebrtc` を `M81.4044@{#11}` に上げる
    - @voluntas
- [UPDATE] `sdl2` を `2.0.12` に上げる
    - @voluntas
- [UPDATE] `cmake` を `3.17.0` に上げる
    - @voluntas
- [ADD] Windows でも `--video-device` を指定できるようにする
    - @msnoigrs
- [ADD] sora モードの引数に `--audio` と `--video` を追加
    - @melpon
- [CHANGE] ルートでの `--port` 引数を削除し、`sora` モードと `test` モードで `--port` を指定する
    - @melpon
- [CHANGE] `sora` モードで `--port` を指定していない場合、`--auto` を指定しなくても自動的に接続する
    - @melpon
- [CHANGE] `--daemon` 引数を削除
    - @melpon
- [CHANGE] `--no-video` と `--no-audio` 引数を `--no-video-device` と `--no-audio-device` に変更
    - @melpon
- [CHANGE] PCMU オーディオコーデックを削除
    - @melpon
- [CHANGE] sora モードの `--video-codec` や `--audio-codec` を指定しなかった場合、Sora 側のデフォルト値を使うようにする
    - 今までは VP8, OPUS だった
    - @melpon
- [FIX] video_adapter_メンバ変数は使用していないので削除する
    - @msnoigrs
- [FIX] Ubuntu 18.04 で `libcuda.so` / `libnvcuvid.so` がイントールされていなくても起動するようにする
    - @melpon

## 2020.3.1

- [FIX] ubuntu-18.04_x86_64 で H.264 を有効にする
    - @melpon

## 2020.3

- [UPDATE] Raspberry Pi の H.264 を利用時のリサイズ処理をハードウェアに変更する
    - VPU でソフトウェア処理される `vc.ril.resize` からハードウェア処理される `vc.ril.isp` への変更
    - YUV の形式が異なる場合の変換処理もハードウェアに変更
    - @tnoho
- [UPDATE] libwebrtc を M81.4044@{#9} に上げる
    - @voluntas
- [UPDATE] libwebrtc を M81.4044@{#7} に上げる
    - @voluntas
- [UPDATE] libwebrtc を M80.3987@{#6} に上げる
    - @voluntas
- [ADD] Windows 10 で NVIDIA VIDEO CODEC SDK を利用した H.264 ハードウェアエンコーダへ対応
    - @melpon
- [ADD] Ubuntu 18.04 で NVIDIA VIDEO CODEC SDK を利用した H.264 ハードウェアエンコーダへ対応
    - @melpon
- [ADD] TLS チェックを行わない --insecure オプションを追加
    - @melpon
- [ADD] WSS と TURN-TLS 時の証明書チェックを libwebrtc ハードコードとデフォルトパス両方を利用するようにする
    - @melpon
- [ADD] WebRTC カスタム用のスクリプトを追加
    - @melpon
- [ADD] Sora モード利用時の `type: pong` で stats 取得して送るようにする
    - @melpon
- [ADD] Raspberry Pi で SDL 利用時に H264 ハードウェアデコーダを利用するようにする
    - @tnoho
- [FIX] Jetson Nano で --use-native を使った際に FHD 設定で下部に緑の帯が出るのを修正
    - https://github.com/shiguredo/momo/issues/124
    - @tetsu-koba  @tnoho
- [FIX] Jetson Nano で H264 デコーダを止める際にハングしてしまう問題を修正
    - @soudegesu @tnoho
- [FIX] macOS で WebRTC のバージョンが埋め込まれていなかった問題を修正
    - @melpon
- [FIX] Jetson Nano で RTP タイムスタンプが 90kHz になっていなかったのを修正
    - https://github.com/shiguredo/momo/pull/137
    - @tetsu-koba @tnoho

## 2020.2.1

**hotfix**

- [FIX] macOS で --use-sdl オプションを利用すると落ちていたのを修正する
    - https://bugzilla.libsdl.org/show_bug.cgi?id=4617
    - @melpon

## 2020.2

- [UPDATE] CLI11 を v1.9.0 にアップデートする
    - @voluntas
- [ADD] Windows 10 対応を追加
    - @melpon
- [ADD] Windows の Sora/Ayame モード利用時のシグナリング接続情報に environment / libwebrtc / sora_client を追加
    - `"environment": "[x64] Windows 10.0 Build 18362"`
    - `"libwebrtc": "Shiguredo-Build M80.3987@{#2} (80.3987.2.1 fba51dc6)"`
    - `"sora_client": "WebRTC Native Client Momo 2020.1 (0ff24ff3)"`
    - @melpon
- [ADD] ビルド環境を CMake 化
    - @melpon
- [CHANGE] ubuntu-18.04_armv8 のビルドを削除
    - @melpon

## 2020.1

- [UPDATE] libwebrtc を M80.3987@{#2} に上げる
    - libwebrtc のハッシュは fba51dc69b97f6f170d9c325a38e05ddd69c8b28
    - @melpon
- [UPDATE] Momo 2020.1 にバージョンを上げる
    - バージョン番号を <リリース年>.<その年のリリース回数> に変更
    - @voluntas
- [UPDATE] Boost 1.72.0 にアップデートする
    - @voluntas
- [UPDATE] --video-device を Linux 全般で有効にする
    - V4L2 capturer を使うようにした
    - @shino
- [UPDATE] Jetson Nano 用のライブラリを NVIDIA L4T 32.3.1 に上げる
    - [L4T \| NVIDIA Developer](https://developer.nvidia.com/embedded/linux-tegra)
    - @melpon
- [UPDATE] 音声系オプションの --disable-residual-echo-detector を追加する
    - @melpon
- [ADD] データチャネルを利用したシリアルポートへの読み書き機能を追加する
    - --serial を指定することでデータチャネル経由でのシリアル読み書きが可能になる
    - test と ayame モードでのみ利用可能
    - @tnoho
- [ADD] 自由に解像度の値を指定できるようにする
    - `--resolution 640x480` のように指定できるようになりました
    - この機能が有効になるのは、カメラに依存するため動作保証はありません
    - @melpon
- [ADD] Sora モード利用時のシグナリング接続情報に enviroment / libwebrtc / sora_client を追加する
    - Jetson Nano の場合
        - `"environment": "[aarch64] Ubuntu 18.04.3 LTS (nvidia-l4t-core 32.2.1-20190812212815)"`
        - `"libwebrtc": "Shiguredo-Build M80.3987@{#2} (80.3987.2.1 15b26e4)"`
        - `"sora_client": "WebRTC Native Client Momo 2020.1 (f6b69e77)"`
    - macOS の場合
        - `"environment": "[x86_64] macOS Version 10.15.2 (Build 19C57)"`
        - `"libwebrtc": "Shiguredo-Build M80.3987@{#2} (80.3987.2.1 15b26e4)"`
        - `"sora_client": "WebRTC Native Client Momo 2020.1 (f6b69e77)"`
    - Ubuntu 18.04 x86_64 の場合
        - `"environment": "[x86_64] Ubuntu 18.04.3 LTS"`
        - `"libwebrtc": "Shiguredo-Build M80.3987@{#2} (80.3987.2.1 15b26e4)"`
        - `"sora_client": "WebRTC Native Client Momo 2020.1 (f6b69e77)"`
    - @melpon
- [ADD] Ayame モード利用時のシグナリング接続情報に enviroment / libwebrtc / ayameClient を追加する
    - Sora 時の sora_client が ayameClient に変わります
    - @melpon
- [ADD] Raspbian ミラーを追加する
    - @melpon
- [CHANGE] momo --help の英語化
    - @shino @msnoigrs
- [CHANGE] <package>.edit の機能とドキュメントを削除
    - @melpon
- [CHANGE] armv6 で SDL を使えなくする
    - @melpon
- [FIX] --no-video を指定しているにもかかわらずカメラを一瞬だけ掴むのを修正する
    - @melpon @mganeko
- [FIX] SDL が有効でない時に SDL 関連のオプションを指定するとエラーにする
    - @melpon
- [FIX] macOS のビルドで Python 2.7 必須を外す
    - @melpon
- [FIX] Ayame モードで WebSocket が閉じられた際に再接続処理に進まない箇所を修正
    - @Hexa
- [FIX] Ayame モードで シグナリングで bye を受信した際処理として、各 close 処理を追加する
    - @Hexa
- [FIX] Ayame モードで 再接続処理の 1 回目を、5 秒後からすぐに実行されるように変更する
    - @Hexa

## 19.12.1

- [UPDATE] libwebrtc を時前ビルドしないようにする
    - https://github.com/shiguredo-webrtc-build/webrtc-build を利用する
    - @melpon
- [FIX] momo + ayame モードで再接続時に delay してしまう問題を解決
    - @kdxu

## 19.12.0

- [UPDATE] libwebrtc M79 コミットポジションを 5 にする
    - libwebrtc のハッシュは b484ec0082948ae086c2ba4142b4d2bf8bc4dd4b
    - @voluntas
- [UPDATE] json を 3.7.3 に上げる
    - @voluntas
- [ADD] sora モード利用時の --role に sendrecv | sendonly | recvonly を指定できるようにする
    - @melpon
- [FIX] QVGA の指定を 320x240 にする
    - @melpon @Bugfire
- [FIX] ayame モードで再接続時に segmentation fault が起こる場合があるのを修正する
   - ただし、互いに接続を確立するまでping-pongを送らない/ping timeoutで再接続するまで数秒かかることがある」ので、再接続によって受信側が数秒待つ必要が出てくる可能性がある
   - 上記の問題はこの修正では未解決
   - @kdxu
- [FIX] OpenH264 を明示的にビルドしないようにする
    - @melpon

## 19.11.1

- [ADD] Raspberry Pi 4 での動作を確認
    - @voluntas @Hexa
- [UPDATE] libwebrtc M79 コミットポジションを 3 にする
    - libwebrtc のハッシュは 2958d0d691526c60f755eaa364abcdbcda6adc39
    - @voluntas
- [UPDATE] libwebrtc M79 コミットポジションを 2 にする
    - libwebrtc のハッシュは 8e36cc906e5e1c16486e60e62acbf79c1c691879
    - @voluntas
- [UPDATE] Ayame で isExistUser フラグが accept 時に返却されなかった場合 2 回 peer connection を生成する
- [ADD] SDL を利用した音声と映像の受信可能にする `--use-sdl` を追加する
    - [Simple DirectMedia Layer](https://www.libsdl.org/)
    - @tnoho
- [ADD] SDL を Sora のマルチストリームに対応する `--multistream` を追加する
    - @tnoho
- [ADD] SDL を Sora のスポットライトに対応する `--spotlight` を追加する
    - @tnoho
- [ADD] SDL 利用時に Jetson Nano では H.264 ハードウェアデコーダを利用するようにする
    - @tnoho
- [ADD] SDL 利用時に自分のカメラ映像を表示する `--show-me` を追加する
    - @tnoho
- [ADD] SDL 利用時に映像を表示するウインドウの幅を `--window-width` と `--window-height` で指定可能にする
    - @tnoho
- [ADD] SDL 利用時に映像を表示するウインドウをフルスクリーンにする `--fullscreen` を追加する
    - f を押すと全画面、もう一度 f を押すと戻る
    - @tnoho
- [ADD] sora 利用時に `--role upstream` または `--role downstream` を指定できるようにする
    - @melpon
- [CHANGE] ayame の `accept` 時に返却される `isExistUser` フラグによって offer を送るかどうかを決めるよう変更する
    - @kdxu
- [FIX] C++14 にする
    - @melpon
- [FIX] USE_H264が定義されない場合でも--video-codecが使えるように修正する
    - @msnoigrs

## 19.11.0

- [UPDATE] json を 3.7.1 に上げる
    - @voluntas
- [UPDATE] GitHub Actions の macOS ビルドを macos-latest に変更する
    - @voluntas
- [UPDATE] libwebrtc M78 コミットポジションを 8 にする
    - libwebrtc のハッシュは 0b2302e5e0418b6716fbc0b3927874fd3a842caf
    - @voluntas
- [ADD] GitHub Actions のデイリービルドに ROS を追加する
    - @voluntas
- [ADD] GitHub Actions のビルドに Jetson Nano と macOS を追加する
    - @voluntas
- [ADD] Jetson Nano で 4K@30 出すためのドキュメントを追加
    - @tnoho @voluntas
- [ADD] macOS 用に --video-device オプションを追加
    - @hakobera
- [FIX] GitHub Actions のビルドがディスク容量不足でエラーになっていたのを修正する
    - @hakobera
- [FIX] ayame の client id を指定していない場合のランダム生成がうまくいっていなかったので修正する
    - @kdxu
- [FIX] ROS バージョンが正常にビルドできていなかったのを修正する
    - @melpon

## 19.09.2

- [UPDATE] libwebrtc M78 コミットポジションを 5 にする
    - libwebrtc のハッシュは dfa0b46737036e347acbd3b47f0f58ff6c8350c8
    - @voluntas
- [FIX] iceServers が json プロパティかつ array の場合のみ ice_servers_ にセットするよう修正する
    - @kdxu

## 19.09.1

- [ADD] Jetson Nano のハードウェアエンコーダを利用する機能を実装
    - @tnoho
- [ADD] Jetson Nano のビルドを追加
    - @melpon
- [ADD] CI を CircleCI から GitHub Actions へ切り替える
    - macOS の時間制限が OSS の場合はないため Weekly build から Daily build のみにきりかえる
    - @hakobera
- [ADD] .clang-format の追加
    - @melpon
- [UPDATE] libwebrtc M78 コミットポジションを 3 にする
    - libwebrtc のハッシュは 68c715dc01cd8cd0ad2726453e7376b5f353fcd1
    - @voluntas
- [UPDATE] コマンドオプションをできるだけ共通化する
    - @melpon
- [UPDATE] Raspberry Pi のビルド OS を Ubuntu 16.04 から 18.04 に上げる
    - @melpon

## 19.09.0

- [ADD] --disable-echo-cancellation オプションを追加
    - @melpon
- [ADD] --disable-auto-gain-control オプションを追加
    - @melpon
- [ADD] --disable-noise-suppression オプションを追加
    - @melpon
- [ADD] --disable-highpass-filter オプションを追加
    - @melpon
- [ADD] --disable-typing-detection オプションを追加
    - @melpon
- [UPDATE] Boost 1.71.0 にアップデートする
    - @voluntas
- [UPDATE] libwebrtc M78 コミットポジションを 0 にする
    - libwebrtc のハッシュは 5b728cca77c46ed47ae589acba676485a957070b
    - @tnoho
- [UPDATE] libwebrtc M77 コミットポジションを 10 にする
    - libwebrtc のハッシュは ad73985e75684cb4ac4dadb9d3d86ad0d66612a0
    - @voluntas
- [FIX] Track を複数の PeerConnection で共有するよう修正
    - @tnoho
- [FIX] --no-audio 設定時にも capturer をチェックしていたので修正
    - @tnoho
- [FIX] PeerConnectionObserver の解放がなかったため修正
    - @tnoho

## 19.08.1

- [ADD] Raspberry Pi 用に `--video-device` オプションを追加
    - @melpon
- [UPDATE] sora の metadata オプションを公開する
    - @melpon

## 19.08.0

- [UPDATE] nlohmann/json を v3.7.0 にアップデートする
    - @melpon
- [UPDATE] Raspbian Buster に対応
    - @voluntas
- [UPDATE] libwebrtc M77 コミットポジションを 6 にする
    - libwebrtc のハッシュは 71e2db7296a26c6d9b18269668d74b764a320680
    - @voluntas
- [UPDATE] libwebrtc M77 コミットポジションを 3 にする
    - libwebrtc のハッシュは 3d8e627cb5893714a66082544d562cbf4a561515
    - @kdxu @voluntas
- [UPDATE] libwebrtc M76 コミットポジションを 3 にする
    - libwebrtc のハッシュは 9863f3d246e2da7a2e1f42bbc5757f6af5ec5682
    - @voluntas
- [UPDATE] I420 の時にもハードウェアでリサイズする
    - @tnoho
- [ADD] Raspberry Pi 向けに --use-native オプションを追加しました
    - USB カメラ用で MJPEG をハードウェアデコードします
    - @tnoho
- [ADD] Raspberry Pi 向けに --force-i420 オプションを追加しました
    - Raspberry Pi 専用カメラ用で MJPEG を使えないため HD 以上の解像度でも MJPEG にせず強制的に I420 でキャプチャーする
    - @tnoho
- [ADD] Ayame のサブコマンドに --signaling-key を追加する
    - @kdxu @tnoho
- [ADD] Ayame 利用時に iceServers の払い出しに対応する
    - 独自の STUN/TURN が利用可能になる
    - @kdxu @tnoho
- [CHANGE] Ayame のサブコマンドで client id を optional に指定できるように修正する
    - @kdxu
- [CHANGE] ./momo p2p を ./momo test に変更する
    - @melpon
- [FIX] Ayame の candidate 交換の際の JSON スキーマが間違っていたのを修正する
    - @kdxu
- [FIX] Ayame の sdp 交換の際の type が answer 固定になっていたのを修正する
    - @kdxu
- [FIX] Ayame で peer connection 生成後に createOffer して send する実装が漏れていたので追加する
    - @kdxu
- [FIX] Ayame で momo を起動したあとに映像を受信できない場合が発生するのバグを修正する
    - @kdxu
- [FIX] Raspberry Pi でハードウェアエンコーダを利用した際に再接続できなくなることがある問題の修正
    - @tnoho
- [FIX] libwebrtc M77 で作成した armv6 バイナリがクラッシュしてしまう問題の対策
    - @tnoho
- [FIX] macOS 版 Momo で VideoToolbox 利用時の解像度変更時に落ちる問題の修正
    - @hakobera
- [FIX] macOS 版がビルドは成功するが動作させようとするとセグメンテーションフォルトする問題の修正
    - @hakobera
- [FIX] Raspberry Pi でハードウェアエンコーダを利用した際にGPUのメモリを食いつぶしてしまう問題の修正
    - @tnoho

## 19.07.0

- [UPDATE] Raspberry Pi の H.264 を MMAL を利用したハードウェアエンコードに変更する
    - 720p 30fps や 1080p 20fps を可能にする
    - @tnoho
- [UPDATE] libwebrtc を M75 に上げる
    - libwebrtc のハッシュは 159c16f3ceea1d02d08d51fc83d843019d773ec6
    - @tnoho
- [UPDATE] libwebrtc を M76 に上げる
    - libwebrtc のハッシュは d91cdbd2dd2969889a1affce28c89b8c0f8bcdb7
    - @kdxu
- [UPDATE] Unified Plan に対応する
    - @tnoho
- [UPDATE] no-audio 時に AudioDevice を無効化するよう変更
    - @tnoho
- [UPDATE] CLI11 を v1.8.0 にアップデートする
    - @melpon
- [UPDATE] JSON v3.6.1 にアップデートする
    - @melpon
- [UPDATE] macOS のビルドドキュメントを独立させる
    - @voluntas
- [UPDATE] doc/CACHE.md を削除
    - make PACKAGE.clean にてビルドキャッシュの削除が可能になったため
    - @melpon
- [UPDATE] audio/video の共通オプションを sora のオプションに移動する
    - Momo 側ではコーデックやビットレートは指定できない
    - p2p の場合は HTML で sdp を切り替えている
    - --audio-codec
    - --audio-bitrate
    - --video-codec
    - --video-bitrate
    - @melpon
- [UPDATE] WebRTC Signaling Server Ayame 19.07.0 に追従する
    - @kdxu
- [ADD] WebRTC Signaling Server Ayame に対応しました
    - https://github.com/OpenAyame/ayame
    - @kdxu
- [ADD] Circle CI で Linux 版を毎日 22:00 に自動ビルドする
    - @voluntas
- [ADD] Circle CI で macOS 版を毎週日曜日 22:00 に自動ビルドする
    - @voluntas
- [FIX] macOS でデバイスがつかめなくなっていたのを修正する
    - ただし --fixed-resolution 必須
    - @tnoho
- [FIX] ROS 対応がビルドできなくなっていたのを修正する
    - @tnoho

## 19.02.0

- [UPDATE] webrtc.js / p2p.html のリファクタリング
- [UPDATE] Momo の前段にリバースプロキシ等を設置して https でアクセス可能にした場合でも、wss で接続できるように webrtc.js を変更する
- [CHANGE] ビルド時のターゲットとオプション、パッケージの作成先を変更する
- [UPDATE] STUN サーバの URL の指定を url から urls に変更する
- [FIX] カメラがない環境で起動させるとセグフォが起きるのを修正する
- [FIX] ARM ROS 版で H.264 配信の場合はハードウェアエンコーダを使用するように修正する
- [CHANGE] ROS Audio に対応する
    - 別ノードから送られてきたオーディオを使用するように変更
- [UPDATE] 利用している libwebrtc のライブラリを M73 にする

## 19.01.0

- [ADD] Ubuntu 16.04 ARMv7 ROS 対応

## 18.12.0

- [ADD] ROS 対応
- [CHANGE] ビルド時のターゲットと、パッケージのファイル名を変更する
- [CHANGE] p2p モードの切断後に自動的に window が閉じられないようにする
- [UPDATE] Boost 1.69.0 にアップデートする
- [UPDATE] CLI11 v1.6.2 にアップデートする
- [UPDATE] JSON v3.4.0 にアップデートする
- [CHANGE] x86_64, armv8 の場合は H264 を指定できないようにする

## 18.10.2

- [ADD] 解像度を固定するオプションを追加する
- [FIX] WebSocket の分かれているべきフレームがくっついていたのを修正した

## 18.10.1

- [UPDATE] 利用している libwebrtc のライブラリを M71 にする
- [FIX] --metadata を Sora のみのオプションにする
- [FIX] P2P のオプションに --document-root を追加する
- [FIX] P2P モードで Web サーバが立ち上がった場合カレントディレクトリを晒さないようにする
- [FIX] --auido-bitrate を指定した場合に、--auido-bitrate に指定したビットレートがビデオのビットレートとして扱われる問題を修正

## 18.10.1-rc0

- [UPDATE] websocketpp と civietweb を Boost.beast に置き換える

## 18.10.0

**18.10.0-rc4 から変更なし**

## 18.10.0-rc4

- [ADD] 4K の配信にに対応する
    - armv6, armv7 にも対応はしているが、現時点で Raspberry Pi で配信はマシンパワー不足のためできない

## 18.10.0-rc3

- [FIX] バージョン情報を MOMO_VERSION に指定したら momo のバイナリの --version も反映するようにする
- [CHANGE] --metadata の引数は JSON のみを指定できるようにする

## 18.10.0-rc2

- [CHANGE] libwebrtc が 4K に対応していないため解像度指定から 4K を削除する
    - 将来的に対応していく予定
    - https://github.com/shiguredo/momo/issues/21
- [FIX] P2P モードのサンプルで映像を有効にした場合、音声が正常に流れない問題を修正

## 18.10.0-rc1

- [UPDATE] ビルド時に `LDFLAGS += -s` 渡してバイナリサイズを圧縮する仕組みを追加
- [ADD] WebRTC が依存しているライブラリのライセンスを追加
- [ADD] Websocketpp が依存しているライブラリのライセンスを追加
- [FIX] P2P モードで画面を開いているときに終了するとセグフォが起きるのを修正

## 18.10.0-rc0

**初 リリース**

- Momo を Apache License 2.0 でオープンソース化
- libwebrtc M70 対応
- Ubuntu 18.04 x86_64 対応
    - Ubuntu 18.04 x86_64 向けのバイナリの提供
- Ubuntu 16.04 ARMv8 対応
    - Ubuntu 16.04 ARMv8 向けのバイナリの提供
    - PINE64 の Rock64 対応
- Raspberry Pi 3 B/B+ 対応
    - Raspberry Pi 3 B/B+ 向け ARMv7 バイナリの提供
    - Raspberry Pi 3 B/B+ 向け H.264 HWA 対応
- Raspberry Pi Zero W/WH 対応
    - Raspberry Pi Zero W/WH 向け ARMv6 バイナリの提供
    - Raspberry Pi Zero W/WH 向け H.264 HWA 対応
- 解像度指定オプション
    - QVGA、VGA、HD、FHD、4K
- フレームレート指定オプション
    - 1-60
- 優先オプション
    - この機能は実験的機能です
    - フレームレートか解像度のどちらを優先するか指定可能
- ビデオを利用しないオプション
- オーディオを利用しないオプション
- ビデオコーデック指定オプション
- オーディオコーデック指定オプション
- デーモン化オプション
    - Systemd の利用をおすすめします
- ログレベル
- P2P 機能
- WebRTC SFU Sora 18.04.12 対応
