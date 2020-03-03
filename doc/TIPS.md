# TIPS

## オレオレ証明書を利用したい

Momo は libwebrtc にハードコードされている証明書以外に OpenSSL インストール先ディレクトリの cert.pem を検索します。

```
デフォルトのルート証明書は、通常 <OpenSSLのインストール先ディレクトリ>/cert.pem を検索します。
ただし、OpenSSL の仕様により、SSL_CERT_FILE 環境変数を設定することでこのデフォルトのルート証明書のパスを変更できます。
```

[ルート証明書の設定を統一してカスタマイズできるようにした by melpon · Pull Request \#135 · shiguredo/momo](https://github.com/shiguredo/momo/pull/135)

