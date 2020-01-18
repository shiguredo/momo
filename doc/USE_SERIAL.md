# データチャネル経由でのシリアル読み書きを使ってみる

**この機能は現在 test と ayame モードでしか利用できません**

ここでは [socat](http://www.dest-unreach.org/socat/) を利用して試してみます。

[socatで仮想シリアルポートを作る \- Qiita](https://qiita.com/uhey22e/items/dc41d7fa1075970e66a1)

socat はインストールされている前提です。

内部的につながっている仮想シリアルポートを作成します。

```
$ socat -d -d pty,raw,echo=0 pty,raw,echo=0
2020/01/18 18:47:21 socat[71342] N PTY is /dev/ttys003
2020/01/18 18:47:21 socat[71342] N PTY is /dev/ttys004
2020/01/18 18:47:21 socat[71342] N starting data transfer loop with FDs [5,5] and [7,7]
```

これで /dev/ttys003 と /dev/ttys004 が内部的につながっていることになります。

ttys003 に Momo を繋ぎます。

```
$ ./momo --serial /dev/ttys003,9600 test
```

/dev/ttys004 に書かれたデータ表示するようにします。

```
$ cat < /dev/ttys004
```

http://127.0.0.1:8080/html/test.html にアクセスします。

send のところでなにか文字列を送って無事 ttys004 経由で表示される事を確認してください。

## 参考動画

[![Image from Gyazo](https://i.gyazo.com/c1fb6696963e044a44576b1ddeffd0cb.gif)](https://gyazo.com/c1fb6696963e044a44576b1ddeffd0cb)


[![Image from Gyazo](https://i.gyazo.com/269ccc2290b43809a0e67e35c03e8601.gif)](https://gyazo.com/269ccc2290b43809a0e67e35c03e8601)
