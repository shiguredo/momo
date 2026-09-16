# Ayame モードを利用して Momo を動かしてみる

Ayame は時雨堂が開発し OSS として公開している、 WebRTC シグナリングサーバーです。

[OpenAyame プロジェクト](https://gist.github.com/voluntas/90cc9686a11de2f1acca845c6278a824)

## Ayame を利用したサービス Ayame Labo を利用する

Ayame を利用してシグナリングサーバーを立てるのが面倒な人向けに Ayame Labo を提供しています。

Ayame Labo は時雨堂が提供している Ayame を利用したサービスです。無料で利用できます。

<https://ayame-labo.shiguredo.app/>

### Ayame Labo にサインアップしない場合

Ayame Labo はサインアップせずにシグナリングサーバーを利用できます。

ここではルーム ID は `open-momo` としていますが、必ず推測されにくい値に変更してください。

```bash
./momo --no-audio-device ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling --room-id open-momo
```

#### Windows の場合

```powershell
.\momo.exe --no-audio-device ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling --room-id open-momo
```

Ayame SDK のオンラインサンプルは利用できないため、Momo どうしでの接続確認を行ってください。

### Ayame Labo にサインアップする場合

Ayame Labo にサインアップした場合はルーム ID に GitHub ユーザー名を先頭に指定する必要があります。
例えば GitHub ユーザー名が `shiguredo` の場合は `shiguredo@open-momo` です。

- ルーム ID に `GitHub ユーザー名` + `@` を先頭に指定する必要があります
  - ここでは GitHub ユーザー名を `shiguredo` として、 `shiguredo@open-momo` をルーム ID としています
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

## 送受信方向の制御

Ayame モードでは `--direction` オプションを使用して、映像 / 音声の送受信方向を制御できます。

### 利用可能な値

- `sendrecv` - 送受信する（デフォルト）
- `sendonly` - 送信専用
- `recvonly` - 受信専用

### 送信する場合

配信用途などで、映像 / 音声を送信したい場合は `--direction sendonly` を指定します。

```bash
./momo --no-audio-device ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling --room-id open-momo --direction sendonly
```

### 受信する場合

視聴用途などで、映像 / 音声を受信したい場合は `--direction recvonly` を指定します。

```bash
./momo --no-audio-device ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling --room-id open-momo --direction recvonly
```

### 送受信する場合

双方向通信する場合は `--direction sendrecv` を指定するか、オプションを省略します。

```bash
./momo --no-audio-device ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling --room-id open-momo --direction sendrecv
```
