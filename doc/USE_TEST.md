# テストモードを利用して Momo を動かしてみる

Momo 自体がシグナリングサーバの機能を持つ test モードを利用して動かしてみてください。

## Momo で配信をしてみる

```shell
./momo --no-audio-device test
```

Windows の場合:

```
.\momo.exe --no-audio-device test
```

Momo の IP アドレスが 192.0.2.100 の場合は、
http://192.0.2.100:8080/html/test.html に Chrome でアクセスして接続してみてください。

## ローカルネットワークの Momo 同士で双方向配信をしてみる

- Momo を搭載しているマシンが同一ネットワーク上にいるか確認してください。
- Momo を搭載している2台のマシンでそれぞれコマンドを実行してください。
- 映像の確認をするためには SDL 機能が必要となります。そのため、 CUI 環境では利用できませんのでご注意ください。
SDL についての詳細は [USE_SDL.md](USE_SDL.md) をお読みください。

Momo 1:

```shell
./momo --use-sdl test
```

Momo 2:

```shell
./momo --use-sdl ayame  --signaling-url ws://[Momo 1 の IP アドレス]:8080/ws --room-id test
```

Google STUN を利用したくない場合は`--no-google-stun`をオプションを追加することで可能になります。

Momo 1:

```shell
./momo --no-google-stun　--use-sdl test
```

Momo 2:

```shell
./momo  --no-google-stun --use-sdl ayame --signaling-url ws://[Momo 1 の IP アドレス]:8080/ws --room-id test
```

配信がうまくいくとそれぞれのマシンにお互いの映像と音声が出力されます。  

## テストモードで確認ができたら

うまく接続できたら、次は Ayame を利用して動かしてみてください。

Ayame を利用する場合は [USE_AYAME.md](USE_AYAME.md) をご確認ください。
