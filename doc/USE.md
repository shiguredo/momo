# Momo を使ってみる

Momo を初めて利用する場合 テストモードでの利用をおすすめします。

## Raspberry Pi で Momo を使ってみる

[USE_RASPBERRY_PI.md](USE_RASPBERRY_PI.md) をお読みください。

## NVIDIA Jetson Nano で Momo を使ってみる

[USE_JETSON_NANO.md](USE_JETSON_NANO.md) をお読みください。

## macOS で Momo を使ってみる

[USE_MAC.md](USE_MAC.md) をお読みください。

## テストモードを利用して Momo を動かしてみる

[USE_TEST.md](USE_TEST.md) をお読みください。

## Ayame モードを 利用して Momo を動かしてみる

Ayame モードでは時雨堂が開発しているオープンソースのシグナリングサーバ [WebRTC Signaling Server Ayame](https://github.com/OpenAyame/ayame) を利用します。

[Ayame Lite](https://ayame-lite.shiguredo.jp/) を利用することで、 無料で試すことが可能です。

[USE_AYAME.md](USE_AYAME.md) をお読みください。

## Sora モードを 利用して Momo を動かしてみる

Sora モードでは時雨堂が開発、販売している WebRTC SFU Sora を利用します。

[Sora Labo](https://sora-labo.shiguredo.jp/) を利用することで、 Sora を無料で試すことが可能です。

[USE_SORA.md](USE_SORA.md) をお読みください。

## SDL を利用した受信機能を使ってみる

Momo では SDL (Simple DirectMedia Layer) を利用して音声や映像を出力することが可能になります。

[USE_SDL.md](USE_SDL.md) をお読みください。

## ROS ノードとして Momo を使ってみる

- Momo を ROS ノードとして使ってみたい人は [USE_ROS.md](USE_ROS.md) をお読みください。
- ARM 対応版の Momo を ROS ノードとして使ってみたい人は [USE_ARM_ROS.md](USE_ARM_ROS.md) をお読みください。

## コマンド

### バージョン情報

```
$ ./momo --version
WebRTC Native Client Momo version 19.11.0 USE_MMAL_ENCODER=0
```

### ヘルプ

```
$ ./momo --help
Momo - WebRTC ネイティブクライアント
Usage: ./momo [OPTIONS] [SUBCOMMAND]

Options:
  -h,--help                   Print this help message and exit
  --no-video                  ビデオを表示しない
  --no-audio                  オーディオを出さない
  --force-i420                強制的にI420にする（対応デバイスのみ）
  --use-native                MJPEGのデコードとビデオのリサイズをハードウェアで行う（対応デバイスのみ）
  --video-device TEXT         デバイス番号、またはデバイス名。省略時はデフォルト（デバイス番号が0）のビデオデバイスを自動検出
  --resolution TEXT:{4K,FHD,HD,QVGA,VGA}
                              解像度
  --framerate INT:INT in [1 - 60]
                              フレームレート
  --fixed-resolution          固定解像度
  --priority TEXT:{BALANCE,FRAMERATE,RESOLUTION}
                              優先設定 (Experimental)
  --port INT:INT in [0 - 65535]
                              ポート番号(デフォルト:8080)
  --use-sdl                   SDLを使い映像を表示する
  --show-me                   自分のカメラも表示する
  --window-width INT:INT in [180 - 16384]
                              映像を表示するウィンドウの横幅
  --window-height INT:INT in [180 - 16384]
                              映像を表示するウィンドウの縦幅
  --fullscreen                映像を表示するウィンドウをフルスクリーンにする
  --daemon                    デーモン化する
  --version                   バージョン情報の表示
  --log-level INT:value in {verbose->0,info->1,warning->2,error->3,none->4} OR {0,1,2,3,4}
                              ログレベル
  --disable-echo-cancellation エコーキャンセルを無効
  --disable-auto-gain-control オートゲインコントロール無効
  --disable-noise-suppression ノイズサプレッション無効
  --disable-highpass-filter   ハイパスフィルター無効
  --disable-typing-detection  タイピングディテクション無効

Subcommands:
  test                        開発向け
  ayame                       WebRTC Signaling Server Ayame
  sora                        WebRTC SFU Sora
```

### test モードヘルプ


```
$ ./momo test --help
開発向け
Usage: ./momo test [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --document-root TEXT:DIR    配信ディレクトリ
```

### ayame モードヘルプ


```
$ ./momo ayame --help
WebRTC Signaling Server Ayame
Usage: ./momo ayame [OPTIONS] SIGNALING-URL ROOM-ID

Positionals:
  SIGNALING-URL TEXT REQUIRED シグナリングホスト
  ROOM-ID TEXT REQUIRED       ルームID

Options:
  -h,--help                   Print this help message and exit
  --client-id TEXT            クライアントID
  --signaling-key TEXT        シグナリングキー
```

### sora モードヘルプ

```
$ ./momo sora --help
WebRTC SFU Sora
Usage: ./momo sora [OPTIONS] SIGNALING-URL CHANNEL-ID

Positionals:
  SIGNALING-URL TEXT REQUIRED シグナリングホスト
  CHANNEL-ID TEXT REQUIRED    チャンネルID

Options:
  -h,--help                   Print this help message and exit
  --auto                      自動接続する
  --video-codec TEXT:{H264,VP8,VP9}
                              ビデオコーデック
  --audio-codec TEXT:{OPUS,PCMU}
                              オーディオコーデック
  --video-bitrate INT:INT in [1 - 30000]
                              ビデオのビットレート
  --audio-bitrate INT:INT in [6 - 510]
                              オーディオのビットレート
  --multistream               マルチストリームかどうか
  --role TEXT:{downstream,upstream}
                              ロール（デフォルトは upstream）
  --spotlight INT:INT in [1 - 10]
                              スポットライトの配信数
  --metadata TEXT:JSON Value  メタデータ
```

## うまく動作しない時

- カメラを uv4l など他のプロセスが利用していないか確認してください

## Q&A

### コーデックの指定やビットレートを利用したい

Momo 側からの指定は WebRTC SFU Sora を利用したときだけ可能です。ただし受信側で指定する指定することで対応可能です。
