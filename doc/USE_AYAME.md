# Ayame モードを 利用して Momo を動かしてみる

Ayame は時雨堂が開発し OSS として公開している、 WebRTC シグナリングサーバです。

[OpenAyame プロジェクト](https://gist.github.com/voluntas/90cc9686a11de2f1acca845c6278a824)

## Ayame を利用したサービス Ayame Labo を利用する

Ayame を利用してシグナリングサーバを立てるのが面倒な人向けに Ayame Labo を提供しています。

Ayame Labo は時雨堂が提供している Ayame を利用したサービスです。無料で利用可能です。

<https://ayame-labo.shiguredo.app/>

### Ayame Labo にサインアップしない場合

Ayame Labo はサインアップせずにシグナリングサーバを利用可能です。

ここではルーム ID は `open-momo` としておりますが、必ず推測されにくい値に変更してください。

```bash
./momo --no-audio-device ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling --room-id open-momo
```

#### Windows の場合

```powershell
.\momo.exe --no-audio-device ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling --room-id open-momo
```

Ayame SDK のオンラインサンプルは利用できないため、momo 同士での接続確認を行ってください。

### Ayame Labo にサインアップする場合

Ayame Labo にサインアップした場合はルーム ID に GitHub ユーザ名を先頭に指定する必要があります。
例えば GitHub ユーザ名が `shiguredo` の場合は `shiguredo@open-momo` となります。

- ルーム ID に `GitHub ユーザ名` + `@` を先頭に指定する必要があります
  - ここでは GitHub ユーザ名を `shiguredo` として、 `shiguredo@open-momo` をルーム ID としています。
- シグナリングキーを `--signaling-key` にて指定する必要があります
  - ここではシグナリングキーを `xyz` としています

```bash
./momo --no-audio-device ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling --room-id shiguredo@open-momo --signaling-key xyz
```

#### Windows の場合

```powershell
.\momo.exe --no-audio-device ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling --room-id shiguredo@open-momo --signaling-key xyz
```

Ayame SDK のオンラインサンプルを利用します。 URL の引数にルーム ID とシグナリングキーを指定してアクセスします。

<https://openayame.github.io/ayame-web-sdk/devtools/index.html?roomId=shiguredo@open-momo&signalingUrl=wss://ayame-labo.shiguredo.app/signaling&signalingKey=xyz>
