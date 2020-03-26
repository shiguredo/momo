# Windows で Momo を使ってみる

## Windows 向けのバイナリは以下にて提供しています

https://github.com/shiguredo/momo/releases にて最新版のバイナリをダウンロードしてください。

## 動かしてみる

動かし方については [USE_TEST.md](USE_TEST.md) を御覧ください。

### PowerShellやコマンドプロンプトで実行する際の注意

PowerShellやコマンドプロンプトで実行する場合文字列エスケープの仕様のため
metadataオプションのキーや値を囲む「"」を「\"」にする必要があります。

例：
```
./momo --no-audio-device sora \
    wss://sora-labo.shiguredo.jp/signaling shiguredo@sora-labo \
        --video-codec VP8 --video-bitrate 500 \
        --audio false \
        --role sendonly --metadata '{\"signaling_key\": \"xyz\"}'
```
