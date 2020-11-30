# SDL を利用した双方向

**この機能は実験的機能です**

## 概要

SDL (Simple DirectMedia Layer) を利用することで、 Momo 自体が受信した映像を出力することができるようになります。

## 注意

- この機能は ayame と sora モードでのみ利用できます
    - test モードでは test.html が HTTPS ではないため getUserMedia を使用できません
- この機能は Windows または macOS または Linux で利用できます
- この機能を以下の組み合わせで利用するとパフォーマンスに影響が出るため避けてください
    - --hw-mjpeg-decoder=true と --show-me を併用する組み合わせ

## SDL コマンド引数

- --use-sdl
    - SDL 機能を使う場合は指定します
- --show-me
    - Momo が取得した映像を出力します
- --window-width
    - 映像を表示するウインドウの横幅を指定します
- --window-height
    - 映像を表示するウインドウの縦幅を指定します
- --fullscreen
    - 映像を表示するウインドウをフルスクリーンにします

### Sora モード

- --multistream
    - Sora でマルチストリーム機能を利用する場合に指定します
- --role upstream または --sora downstream
    - Sora でロールを切り替える場合に指定します。デフォルトは upstream で、受信専用にする場合は downstream を指定します。
- --spotlight
    - Sora でスポットライト機能を利用する場合に指定します

## Ayame を利用した 1:1 の双方向

- ルーム ID を推測されにくい値に変更して下さい

```
./momo --resolution VGA --no-audio-device --use-sdl --show-me ayame wss://ayame-labo.shiguredo.jp/signaling momo-sdl-ayame
```

[![Image from Gyazo](https://i.gyazo.com/8ca80e9b60c9e848e04afcefd86a2c07.png)](https://gyazo.com/8ca80e9b60c9e848e04afcefd86a2c07)

## Sora を利用したマルチストリームでの双方向


- Signaling サーバの URL はダミーです

```
./momo --resolution VGA --no-audio-device --use-sdl --show-me sora --video-codec VP8 --video-bitrate 1000 --audio false wss://example.com/signaling momo-sdl-sora --multistream
```

[![Image from Gyazo](https://i.gyazo.com/abdb1802bd66440ef32e75da6842f0cf.png)](https://gyazo.com/abdb1802bd66440ef32e75da6842f0cf)


## 全画面

- f を押すと全画面になります、もう一度 f を押すと戻ります
- q を押すと Momo 自体を終了します
