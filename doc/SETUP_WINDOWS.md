# Windows で Momo を使ってみる

## Windows 向けのバイナリは以下にて提供しています

https://github.com/shiguredo/momo/releases にて最新版のバイナリをダウンロードしてください。

## 動かしてみる

動かし方については [USE_TEST.md](USE_TEST.md) を御覧ください。

### PowerShellやコマンドプロンプトで実行する際の注意

PowerShellやコマンドプロンプトで実行する場合文字列エスケープの仕様のため
metadataオプションのキーや値を囲む「"」を「\\\"」にする必要があります。

PowerShell での実行例：
```
.\momo.exe --no-audio-device `
    sora `
        --signaling-url `
             wss://0001.canary.sora-labo.shiguredo.app/signaling `
             wss://0002.canary.sora-labo.shiguredo.app/signaling `
             wss://0003.canary.sora-labo.shiguredo.app/signaling `
        --channel-id sora@shiguredo#0 `
        --video-codec-type VP8 --video-bit-rate 500 `
        --audio false `
        --role sendonly --metadata '{\"access_token\": \"xyz\"}'
```
