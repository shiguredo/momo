# WebRTC SFU Sora を使って Momo を動かしてみる

Sora は時雨堂が開発、販売している商用 WebRTC SFU 製品です。

https://sora.shiguredo.jp/

ここでは利用申請することで法人などで無料で検証可能な [Sora Labo](https://sora-labo.shiguredo.app/) を利用しています。

Sora Labo の利用申請や使用方法については [Sora Labo のドキュメント](https://github.com/shiguredo/sora-labo-doc)をご確認ください。

## Sora Labo を使う

GitHub アカウントを用意して https://sora-labo.shiguredo.app/ にサインアップしてください。

### 片方向配信をしてみる

- チャネル名に `<好きな文字列>@<github-username>#<github-id>` を指定してください
    - 好きな文字列が sora 、GitHub ID が 0 、 GitHub ユーザ名が shiguredo とした場合は `shiguredo_0_sora` のように指定してください
    - ここではチャネル ID を `shiguredo_0_sora` とします
- sora モードのオプションである --metadata を利用し生成したアクセストークンを `access_token` で指定します
    - SoraLabo Home のアクセストークン生成にて先程チャネル名で指定した `<好きな文字列>` を入力してアクセストークンを生成してください
    - この指定は商用の Sora を利用する場合は不要です。Sora Labo 専用の機能になります
    - ここではアクセストークンを `xyz` とします。

```shell
./momo --no-audio-device \
    sora \
        --signaling-url \
            wss://0001.canary.sora-labo.shiguredo.app/signaling \
            wss://0002.canary.sora-labo.shiguredo.app/signaling \
            wss://0003.canary.sora-labo.shiguredo.app/signaling \
        --channel-id shiguredo_0_sora \
        --video-codec-type VP8 --video-bit-rate 500 \
        --audio false \
        --role sendonly --metadata '{"access_token": "xyz"}'
```

ブラウザでの送受信は Sora Labo にあるサンプルのマルチストリーム受信を利用して確認してください。

### 双方向配信をしてみる

GUI 環境で Momo を利用すると、 SDL を利用し音声や映像の受信が可能になります。

```shell
./momo --resolution VGA --no-audio-device --use-sdl --show-me \
    sora \
        --signaling-url \
            wss://0001.canary.sora-labo.shiguredo.app/signaling \
            wss://0002.canary.sora-labo.shiguredo.app/signaling \
            wss://0003.canary.sora-labo.shiguredo.app/signaling \
        --channel-id shiguredo_0_sora \
        --video-codec-type VP8 --video-bit-rate 1000 \
        --audio false \
        --role sendrecv --metadata '{"access_token": "xyz"}'
```

ブラウザでの送受信は Sora Labo にあるサンプルのマルチストリーム送受信を利用して確認してください。

### サイマルキャストで配信してみる

```shell
./momo --no-audio-device \
    sora \
        --signaling-url \
            wss://0001.canary.sora-labo.shiguredo.app/signaling \
            wss://0002.canary.sora-labo.shiguredo.app/signaling \
            wss://0003.canary.sora-labo.shiguredo.app/signaling \
        --channel-id shiguredo_0_sora \
        --video-codec-type VP8 --video-bit-rate 500 \
        --audio false \
        --simulcast true \
        --role sendonly --metadata '{"access_token": "xyz"}'
```

ブラウザでの送受信は Sora Labo にあるサンプルのマルチストリームサイマルキャスト受信を利用して確認してください。
