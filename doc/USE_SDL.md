# SDL を利用した双方向

**この機能は実験的機能です**

## 概要

SDL (Simple DirectMedia Layer) を利用することで、 Momo 自体が受信した映像を出力することができるようになります。

## 注意

- この機能は ayame と sora モードでのみ利用できます
  - p2p モードでは p2p.html が HTTPS ではないため getUserMedia を使用できません
- この機能は Windows または macOS または Linux で利用できます

## SDL コマンド引数

- --use-sdl
  - SDL 機能を使う場合は指定します
- --window-width
  - 映像を表示するウインドウの横幅を指定します
- --window-height
  - 映像を表示するウインドウの縦幅を指定します
- --fullscreen
  - 映像を表示するウインドウをフルスクリーンにします

### Sora モード

- --role sendonly, --sora recvonly または --sora sendrecv
  - Sora でロールを切り替える場合に指定します。送信専用にする場合は sendonly で、受信専用にする場合は recvonly、送受信する場合は sendrecv を指定します。sendrecv はマルチストリームの場合のみ利用可能です。デフォルトは sendonly です。
- --spotlight
  - Sora でスポットライト機能を利用する場合に指定します

## Ayame を利用した 1:1 の双方向

- Ayame Labo にサインアップしない場合の例です
- ルーム ID を推測されにくい値に変更して下さい
- momo 同士で 2 接続した場合の例です

```bash
./momo --resolution VGA --no-audio-device --use-sdl ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling --room-id momo-sdl-ayame
```

[![Image from Gyazo](https://i.gyazo.com/4fca01c1b92ff60f519b3e6c7941ed19.png)](https://gyazo.com/4fca01c1b92ff60f519b3e6c7941ed19)

## Sora を利用したマルチストリームでの双方向

- Signaling サーバの URL はダミーです

```bash
./momo --resolution VGA --no-audio-device --use-sdl sora --role sendrecv --video-codec-type VP8 --video-bit-rate 1000 --audio false --signaling-urls wss://example.com/signaling --channel-id momo-sdl-sora
```

[![Image from Gyazo](https://i.gyazo.com/e0c864b2e0a04fde210a2013ed634a53.png)](https://gyazo.com/e0c864b2e0a04fde210a2013ed634a53)

## 全画面

- f を押すと全画面になります、もう一度 f を押すと戻ります
- q を押すと Momo 自体を終了します
