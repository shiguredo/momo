# テストモードで動かしてみる


Momo 自体がシグナリングサーバの機能を持つ test モードを利用して動かしてみてください。

```shell
$ ./momo --no-audio test
```

Windows の場合:

```
$ .\momo.exe --no-audio test
```

momo の IP アドレスが 192.0.2.100 の場合は、
http://192.0.2.100:8080/html/test.html に Chrome でアクセスして接続してみてください。

うまく接続できたら、次は Ayame を利用して動かしてみてください。

Ayame を利用する場合は [USE_AYAME.md](USE_AYAME.md) をご確認ください。
