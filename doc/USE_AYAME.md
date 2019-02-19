# Ayame で Momo を使ってみる

## コマンド

```
$ ./momo ayame --help
WebRTC Signaling Server Ayame
Usage: ./momo ayame [OPTIONS] SIGNALING-URL ROOM-ID

Positionals:
  SIGNALING-URL TEXT REQUIRED シグナリングホスト
  ROOM-ID TEXT REQUIRED       ルーム ID

Options:
  -k,--key                    キー
  -h,--help                   Print this help message and exit
```

## Ayame で動作を確認する

```
$ ./momo --no-audio --video-codec VP8 --video-bitrate 500 \
         ayame wss://example.com/ws open-ayame
```
