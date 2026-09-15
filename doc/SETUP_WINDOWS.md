# Windows で Momo を使ってみる

## Windows 向けのバイナリは以下にて提供しています

<https://github.com/shiguredo/momo/releases> にて最新版のバイナリをダウンロードしてください。

## 動かしてみる

動かし方については [USE_P2P.md](USE_P2P.md) を確認してください。

### Sora モードで `--metadata` を指定するとき

`--metadata` には JSON を渡します。値は `{"access_token": "xyz"}` の形です。

PowerShell では JSON 全体を単引用符で囲み、内側の `"` は `\"` と書きます。`{` がスクリプトブロックとして解釈されないようにするためです。

PowerShell での実行例：

```powershell
.\momo.exe --no-audio-device `
    sora `
        --signaling-urls `
             wss://sora.sora-labo.shiguredo.app/signaling `
        --channel-id shiguredo_0_sora `
        --video-codec-type VP8 --video-bit-rate 500 `
        --audio false `
        --role sendonly --metadata '{\"access_token\": \"xyz\"}'
```

コマンドプロンプトでは、JSON 全体を二重引用符で囲み、内側の `"` を `\"` にします。行継続は行末の `^` です。PowerShell の単引用符とは外側の囲み方が違います。

コマンドプロンプトでの実行例：

```batch
.\momo.exe --no-audio-device ^
    sora ^
        --signaling-urls ^
             wss://sora.sora-labo.shiguredo.app/signaling ^
        --channel-id shiguredo_0_sora ^
        --video-codec-type VP8 --video-bit-rate 500 ^
        --audio false ^
        --role sendonly --metadata "{\"access_token\": \"xyz\"}"
```
