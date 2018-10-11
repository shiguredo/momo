# 開発者向けメモ

## パッケージング

事前にバイナリを生成しておき、 build ディレクトリで make pkg とすることでパッケージング可能です。

## サブモジュール

```
$ git submodule status
 da901cca542612a133efcb04e8e78080186991e4 libs/CLI11 (v1.6.1-8-gda901cc)
 0f1b43536d97d9311c73b658b86a9d44be9e5e82 libs/civetweb (v1.10)
 d2dd27dc3b8472dbaa7d66f83619b3ebcd9185fe libs/json (v3.1.2)
 c6d7e295bf5a0ab9b5f896720cc1a0e0fdc397a7 libs/websocketpp (0.3.0-395-gc6d7e29)
```
