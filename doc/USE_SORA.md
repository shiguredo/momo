# WebRTC SFU Sora を使って Momo を動かしてみる

Sora は時雨堂が開発、販売している商用 WebRTC SFU 製品です。

https://sora.shiguredo.jp/

ここでは利用申請することで法人などで無料で検証可能な [Sora Labo](https://sora-labo.shiguredo.app/) を利用しています。

Sora Labo の利用申請や使用方法については [Sora Labo のドキュメント](https://github.com/shiguredo/sora-labo-doc)をご確認ください。

## Sora Labo を使う

GitHub アカウントを用意して https://sora-labo.shiguredo.app/ にサインアップしてください。

### 片方向配信をしてみる

- チャネル ID に `<GitHub ユーザ名>@<好きな文字列(最大255 バイトまで)>` を指定してください
    - GitHub ユーザ名が shiguredo とした場合は `shiguredo@sora-labo` のように指定してください
    - ここではチャネル ID を `shiguredo@sora-labo` とします
- sora モードのオプションである --metadata を利用し取得したシグナリングキーを `signaling_key` を指定します
    - この指定は商用の Sora を利用する場合は不要です。Sora Labo 専用の機能になります
    - ここではシグナリングキーを `xyz` とします。

```shell
./momo --no-audio-device \
    sora \
        --signaling-url \
            wss://<IPv4Address>.<ClusterName>.sora.sora-labo.shiguredo.app/signaling \
            wss://<IPv4Address>.<ClusterName>.sora.sora-labo.shiguredo.app/signaling \
            wss://<IPv4Address>.<ClusterName>.sora.sora-labo.shiguredo.app/signaling \
        --channel-id shiguredo@sora-labo \
        --video-codec-type VP8 --video-bit-rate 500 \
        --audio false \
        --multistream true \
        --role sendonly --metadata '{"signaling_key": "xyz"}'
```

ブラウザでの送受信は Sora Labo にあるサンプルのマルチストリーム受信を利用して確認してください。

### 双方向配信をしてみる

GUI 環境で Momo を利用すると、 SDL を利用し音声や映像の受信が可能になります。

```shell
./momo --resolution VGA --no-audio-device --use-sdl --show-me \
    sora \
        --signaling-url \
            wss://<IPv4Address>.<ClusterName>.sora.sora-labo.shiguredo.app/signaling \
            wss://<IPv4Address>.<ClusterName>.sora.sora-labo.shiguredo.app/signaling \
            wss://<IPv4Address>.<ClusterName>.sora.sora-labo.shiguredo.app/signaling \
        --channel-id shiguredo@open-momo \
        --video-codec-type VP8 --video-bit-rate 1000 \
        --audio false \
        --multistream true \
        --role sendrecv --metadata '{"signaling_key": "xyz"}'
```

ブラウザでの送受信は Sora Labo にあるサンプルのマルチストリーム送受信を利用して確認してください。

### サイマルキャストで配信してみる

```shell
./momo --no-audio-device \
    sora \
        --signaling-url \
            wss://<IPv4Address>.<ClusterName>.sora.sora-labo.shiguredo.app/signaling \
            wss://<IPv4Address>.<ClusterName>.sora.sora-labo.shiguredo.app/signaling \
            wss://<IPv4Address>.<ClusterName>.sora.sora-labo.shiguredo.app/signaling \
        --channel-id shiguredo@sora-labo \
        --video-codec-type VP8 --video-bit-rate 500 \
        --audio false \
        --multistream true \
        --simulcast true \
        --role sendonly --metadata '{"signaling_key": "xyz"}'
```

ブラウザでの送受信は Sora Labo にあるサンプルのマルチストリームサイマルキャスト受信を利用して確認してください。
