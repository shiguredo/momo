# SDL を利用した双方向

## 概要

SDL (Simple DirectMedia Layer) を利用することで、 Momo 自体が受信した音声や映像を出力することができるようになります。

## 注意

- この機能は ayame と sora モードでのみ利用できます
- この機能は macOS または Linux でのみ利用できます

  - 当たり前ですが CUI 環境では利用できませんのでご注意下さい

## SDL コマンド引数

- --use-sdl
    - SDL 機能を使う場合は指定します
- --show-me
    - Momo が取得した映像を出力します

### Sora モード

- --multistream
    - Sora でマルチストリーム機能を利用する場合は指定します

## Ayame を利用した 1:1 の双方向

- ルーム ID を推測されにくい値に変更して下さい

```
./momo --resolution VGA --no-audio --port 0 --use-sdl --show-me ayame wss://ayame-lite.shiguredo.jp/signaling momo-sdl-ayame
```

[![Image from Gyazo](https://i.gyazo.com/4d7f0b3f3445bb2934fb693f286cad4b.png)](https://gyazo.com/4d7f0b3f3445bb2934fb693f286cad4b)

## Sora を利用したマルチストリームでの双方向


- Signaling サーバの URL はダミーです

```
./momo --resolution VGA --no-audio --port 0 --use-sdl --show-me sora --auto --video-codec VP8 --video-bitrate 1000 wss://example.com/signaling momo-sdl-sora --multistream
```

[![Image from Gyazo](https://i.gyazo.com/abdb1802bd66440ef32e75da6842f0cf.png)](https://gyazo.com/abdb1802bd66440ef32e75da6842f0cf)
