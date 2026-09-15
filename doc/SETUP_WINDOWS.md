# Windows で Momo を使ってみる

## Windows 向けのバイナリは以下にて提供しています

<https://github.com/shiguredo/momo/releases> にて最新版のバイナリをダウンロードしてください。

## 動かしてみる

動かし方については [USE_P2P.md](USE_P2P.md) を確認してください。

### Sora モードで `--metadata` を指定するとき

`--metadata` には JSON を渡します。momo の `argv` に届く文字列は `{"access_token": "xyz"}` の形である必要があります。

PowerShell からネイティブの `momo.exe` を呼ぶとき、単引用符の中身はそのまま `argv` にはなりません。PowerShell がコマンドラインを組み立て、C ランタイムがそれを分割します。そのため JSON の `"` は、単引用符の内側では `\"` と書きます。`{"access_token": "xyz"}` を単引用符で囲んだだけでは、コマンドライン上で引用が途切れ `is not JSON Value` になります。`{` をスクリプトブロックにしないため、全体は単引用符で囲みます。

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
