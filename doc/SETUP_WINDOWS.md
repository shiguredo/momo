# Windows で Momo を使ってみる

## Windows 向けのバイナリは以下にて提供しています

<https://github.com/shiguredo/momo/releases> にて最新版のバイナリをダウンロードしてください。

## 動かしてみる

動かし方については [USE_P2P.md](USE_P2P.md) を確認してください。

### Sora モードで `--metadata` を指定するとき

`--metadata` には JSON を渡します。momo に届く文字列は `{"access_token": "xyz"}` の形である必要があります。

PowerShell では `{` がスクリプトブロックとして解釈されるため、JSON 全体を単引用符で囲みます。キーや値の `"` を `\\\"` に置き換える必要はありません。

PowerShell での実行例：

```powershell
.\momo.exe --no-audio-device `
    sora `
        --signaling-urls `
             wss://canary.sora-labo.shiguredo.app/signaling `
        --channel-id shiguredo_0_sora `
        --video-codec-type VP8 --video-bit-rate 500 `
        --audio false `
        --role sendonly --metadata '{"access_token": "xyz"}'
```

コマンドプロンプトでは、JSON 全体を二重引用符で囲み、内側の `"` を `\"` にします。行継続は行末の `^` です。

コマンドプロンプトでの実行例：

```batch
.\momo.exe --no-audio-device ^
    sora ^
        --signaling-urls ^
             wss://canary.sora-labo.shiguredo.app/signaling ^
        --channel-id shiguredo_0_sora ^
        --video-codec-type VP8 --video-bit-rate 500 ^
        --audio false ^
        --role sendonly --metadata "{\"access_token\": \"xyz\"}"
```
