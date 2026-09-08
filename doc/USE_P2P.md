# P2P モードを利用して Momo を動かしてみる

Momo 自体がシグナリングサーバの機能を持つ P2P モードを利用して動かしてみてください。

## Momo で配信をしてみる

```bash
./momo --no-audio-device p2p
```

Windows の場合:

```powershell
.\momo.exe --no-audio-device p2p
```

Momo の IP アドレスが 192.0.2.100 の場合は、
<http://192.0.2.100:8080/html/p2p.html> に Chrome でアクセスして接続してみてください。

## ローカルネットワークの Momo どうしで双方向配信をしてみる

P2P モードは `ws://<IP>:8080/ws` で Ayame 互換のシグナリングを受け付けます。
1 台目を P2P モード、 2 台目を `ayame` サブコマンドでそのエンドポイントへ接続します。

受信した映像と音声を各マシンで表示するには `--use-sdl` を指定します。
CUI 環境では表示できません。SDL の詳細は [USE_SDL.md](USE_SDL.md) を参照してください。

2 台が同一ネットワーク上にあることを確認したうえで、次のコマンドを実行してください。

Momo 1:

```bash
./momo --use-sdl p2p
```

Momo 2:

```bash
./momo --use-sdl ayame  --signaling-url ws://[Momo 1 の IP アドレス]:8080/ws --room-id p2p
```

Google STUN を利用したくない場合は`--no-google-stun`をオプションを追加することでできるようになります。

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
