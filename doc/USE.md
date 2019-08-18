# Momo を使ってみる

## Raspberry Pi で Momo を使ってみる

[USE_RASPBERRY_PI.md](USE_RASPBERRY_PI.md) をお読みください。

## Mac で Momo を使ってみる

[USE_MAC.md](USE_MAC.md) をお読みください。

## WebRTC Signaling Server Ayame を使って Momo を動かしてみる

時雨堂が開発しているオープンソースのシグナリングサーバ [WebRTC Signaling Server Ayame](https://github.com/OpenAyame/ayame) を利用します。

[USE_AYAME.md](USE_AYAME.md) をお読みください。

## WebRTC SFU Sora を使って Momo を動かしてみる

時雨堂が開発、販売している商用の WebRTC SFU Sora を利用します。

この機能を利用する場合は事前に Sora のライセンスを購入する必要がありますので、ご注意ください。

[USE_SORA.md](USE_SORA.md) をお読みください。

## ROS ノードとして Momo を使ってみる

- Momo を ROS ノードとして使ってみたい人は [USE_ROS.md](USE_ROS.md) をお読みください。
- ARM 対応版の Momo を ROS ノードとして使ってみたい人は [USE_ARM_ROS.md](USE_ARM_ROS.md) をお読みください。

## コマンド

### バージョン情報

```
$ ./momo --version
WebRTC Native Client Momo 19.08.0
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
  --resolution TEXT:{4K,FHD,HD,QVGA,VGA}
                              解像度
  --framerate INT:INT in [1 - 60]
                              フレームレート
  --fixed-resolution          固定解像度
  --priority TEXT:{BALANCE,FRAMERATE,RESOLUTION}
                              優先設定 (Experimental)
  --port INT:INT in [0 - 65535]
                              ポート番号(デフォルト:8080)
  --daemon                    デーモン化する
  --version                   バージョン情報の表示
  --log-level INT:value in {verbose->0,info->1,warning->2,error->3,none->4} OR {0,1,2,3,4}
                              ログレベル

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
```

## うまく動作しない時

- カメラを uv4l など他のプロセスが利用していないか確認してください

## Q&A

### コーデックの指定やビットレートを利用したい

Momo 側からの指定は WebRTC SFU Sora を利用したときだけ可能です。ただし受信側で指定する指定することで対応可能です。
