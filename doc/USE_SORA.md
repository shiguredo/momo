# WebRTC SFU Sora を使って Momo を動かしてみる

ここでは [Sora Labo](https://sora-labo.shiguredo.jp/) は無料で Sora を試すことのできるサービスである Sora Labo を使った例を載せています。

## Sora Labo を使う

GitHub アカウントを用意して https://sora-labo.shiguredo.jp/ にサインアップしてください。

--metadata を利用し取得したシグナリングキーを signaling_key を指定することで簡単に利用できます。

```shell
$ ./momo --no-audio --port 0 sora --auto --video-codec VP8 --video-bitrate 500 wss://sora-labo.shiguredo.jp/signaling sora-labo-open-momo --metadata '{"signaling_key": "ここにシグナリングキーを指定して下さい"}'
```

### マルチストリームで使ってみる

以下の OS の環境で GUI で使うことで送受信が可能です。

- macOS 15.10
- Ubuntu 18.04 Jetson Nano
- Raspbian

```
./momo --resolution VGA --no-audio --port 0 --use-sdl --show-me sora --auto --video-codec VP8 --video-bitrate 1000 wss://sora-labo.shiguredo.jp/signaling momo-sdl-sora --multistream --role upstream --metadata '{"signaling_key": "ここにシグナリングキーを指定して下さい"}'
```
