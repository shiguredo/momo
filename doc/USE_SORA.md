# WebRTC SFU Sora を使って Momo を動かしてみる

ここでは無料で Sora を試すことのできる [Sora Labo](https://sora-labo.shiguredo.jp/) を使います。

## Sora Labo を使う

GitHub アカウントを用意して https://sora-labo.shiguredo.jp/ にサインアップしてください。

### 配信してみる

- チャネル ID に `<GitHub ユーザ名>@<好きな文字列(最大255 バイトまで)>` を指定してください
    - GitHub ユーザ名が shiguredo とした場合は `shiguredo@sora-labo` のように指定してください
    - ここではチャネル ID を `shiguredo@sora-labo` とします
- sora モードのオプションである --metadata を利用し取得したシグナリングキーを `signaling_key` を指定します
    - この指定は商用の Sora を利用する場合は不要です。Sora Labo 専用の機能になります
    - ここではシグナリングキーを `xyz` とします。

```shell
./momo --no-audio --port 0 sora \
    wss://sora-labo.shiguredo.jp/signaling shiguredo@sora-labo \
        --auto --video-codec VP8 --video-bitrate 500 \
        --role sendonly --metadata '{"signaling_key": "xyz"}'
```

ブラウザでの受信は Chrome や Firefox で以下の URL で確認できます。

https://sora-labo.shiguredo.jp/recvonly?videoCodecType=VP8&audio=False

### マルチストリームを使ってみる

以下の OS の環境で GUI で使うことで送受信が可能です。

- macOS 15.10
- Ubuntu 18.04 Jetson Nano
- Raspbian

```shell
./momo --resolution VGA --no-audio --port 0 --use-sdl --show-me \
    sora wss://sora-labo.shiguredo.jp/signaling shiguredo@open-momo \
        --auto --video-codec VP8 --video-bitrate 1000 \
        --multistream --role sendrecv --metadata '{"signaling_key": "xyz"}'
```

ブラウザでの送受信は Chrome や Firefox で以下の URL で確認できます。

https://sora-labo.shiguredo.jp/multi_sendrecv?videoCodecType=VP8&audio=false

