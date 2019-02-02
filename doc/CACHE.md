# Docker のキャッシュについて

基本的に build ディレクトリで `make <パッケージ名>.clean` を実行すれば、Momo バイナリ、中間ファイル、Docker イメージなどが削除されますが、Docker のキャッシュは削除されません。
Momo はビルドでHDDの容量を数十GB取るので、必要に応じてこのキャッシュを削除して下さい。

`make <パッケージ名>.clean` を実行した後、以下のコマンドを実行するとキャッシュを削除できます。

```
docker builder prune
```

ただし `docker builder prune` は全ての参照されていない Docker キャッシュを削除しようとするので、**Momo と無関係なキャッシュも消える可能性があります**。
残念ながら Docker には Momo のキャッシュだけを機械的に消す方法は無さそうなので、無関係なキャッシュが消えるのが問題である場合、以下の方法でうまく削除して下さい。

どのようなキャッシュが存在するかは、以下のコマンドで確認できます。

```
docker system df -v
```

出力例:

```
(snip)

Build cache usage: 12.94GB

CACHE ID            CACHE TYPE          SIZE                CREATED             LAST USED           USAGE               SHARED
6659c036b8dc        internal            880kB               10 days ago         4 hours ago         62                  false
kx16k0mwg3mh        regular             51B                 4 hours ago         4 hours ago         1                   false
smul1qis4c9t        regular             6.94kB              4 hours ago         4 hours ago         1                   false
rj1btzklh4li        regular             282B                4 hours ago         4 hours ago         1                   false
gtekqh875iyk        exec.cachemount     12.9GB              3 days ago          4 hours ago         5                   false
0de2edf7bff4        regular             117MB               6 hours ago         3 hours ago         20                  true
fc94d17b18c9        regular             745B                6 hours ago         3 hours ago         20                  true
4f9c6186221c        regular             0B                  6 hours ago         3 hours ago         20                  true
7eb46ebc7369        regular             7B                  6 hours ago         3 hours ago         21                  true
mntdswqy7c0r        regular             51B                 6 hours ago         3 hours ago         20                  true
wtkhzmtvtfjh        regular             6.94kB              6 hours ago         3 hours ago         20                  true
z0mqs4dz7ywa        regular             282B                6 hours ago         3 hours ago         20                  true
f9h0p4s0e98j        regular             5.81GB              6 hours ago         3 hours ago         20                  true
p80kiv37qvkb        regular             1.6kB               6 hours ago         3 hours ago         20                  true
qts09pk4y8m7        regular             14.3GB              5 hours ago         3 hours ago         20                  true
zitncr65nd90        regular             103MB               5 hours ago         3 hours ago         20                  true
ts1skortn59s        source.local        2.4kB               3 days ago          3 hours ago         24                  false
8c934103363f        frontend            16.3MB              10 days ago         3 hours ago         123                 false
7pjkucu6w2c4        source.local        20.8kB              3 days ago          3 hours ago         24                  false
u80vk9yt7sbp        source.local        0B                  3 days ago          3 hours ago         24                  false
eua2kohr9cnh        regular             823MB               5 hours ago         3 hours ago         20                  true
```

この中から特定のキャッシュだけ削除するには、以下のようなコマンドを実行します。

```
docker builder prune --filter id=gtekqh875iyk
```

これで CACHE ID が gtekqh875iyk のキャッシュだけ削除できます。

あるいは、以下のように書くことで 24 時間以上利用されていない（LAST USED が 24 時間以上前になっている）キャッシュを削除できます。

```
docker builder prune --filter unused-for=24h
```
