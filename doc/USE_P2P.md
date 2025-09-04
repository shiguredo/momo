# P2P モードを利用して Momo を動かしてみる

Momo 自体がシグナリングサーバの機能を持つ p2p モードを利用して動かしてみてください。

## Momo で配信をしてみる

```bash
./momo --no-audio-device p2p
```

Windows の場合:

```
.\momo.exe --no-audio-device p2p
```

Momo の IP アドレスが 192.0.2.100 の場合は、
<http://192.0.2.100:8080/html/p2p.html> に Chrome でアクセスして接続してみてください。

## ローカルネットワークの Momo 同士で双方向配信をしてみる

- Momo を搭載しているマシンが同一ネットワーク上にいるか確認してください。
- Momo を搭載している2台のマシンでそれぞれコマンドを実行してください。
- 映像の確認をするためには SDL 機能が必要となります。そのため、 CUI 環境では利用できませんのでご注意ください。
SDL についての詳細は [USE_SDL.md](USE_SDL.md) をお読みください。

Momo 1:

```bash
./momo --use-sdl p2p
```

Momo 2:

```bash
./momo --use-sdl ayame  --signaling-url ws://[Momo 1 の IP アドレス]:8080/ws --room-id p2p
```

Google STUN を利用したくない場合は`--no-google-stun`をオプションを追加することで可能になります。

Momo 1:

```bash
./momo --no-google-stun　--use-sdl p2p
```

Momo 2:

```bash
./momo  --no-google-stun --use-sdl ayame --signaling-url ws://[Momo 1 の IP アドレス]:8080/ws --room-id p2p
```

配信がうまくいくとそれぞれのマシンにお互いの映像と音声が出力されます。  

## P2P モードで確認ができたら

うまく接続できたら、次は Ayame を利用して動かしてみてください。

Ayame を利用する場合は [USE_AYAME.md](USE_AYAME.md) をご確認ください。
