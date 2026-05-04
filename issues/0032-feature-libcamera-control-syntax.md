## --libcamera-control の構文を momo (C++) 互換に変更する

Created: 2026-05-04
Model: Opus 4.7

## 概要

`--libcamera-control` の構文が momo (C++) と非互換。momo-rs は `KEY=VALUE` 形式、momo は `key value` (空白区切り、複数回指定) 形式。CLI 互換性を保つため momo に合わせる。

## 後方互換性

これは後方互換のない変更 (`feature/change-` ブランチで対応する)。`KEY=VALUE` 形式で指定していた既存ユーザーは新しい構文に移行する必要がある。

## 現状

momo-rs (`src/main.rs:96-114`):

```
--libcamera-control AfMode=0
--libcamera-control AeEnable=false
```

KEY=VALUE 形式でない場合はエラー (`--libcamera-control の値は KEY=VALUE 形式で指定してください`)。

## momo (C++)

`src/util.cpp` の `--libcamera-control`:

```
--libcamera-control AfMode 0
--libcamera-control AeEnable false
```

CLI11 の `allow_extra_args` で空白区切りの `key value` ペアを受け取る。

## 必要な実装

- `--libcamera-control` の構文を `key value` (2 引数) に変更する
- 複数回指定で複数のコントロールを設定できるようにする
- パース後の構造はキーバリューペアの `Vec<(String, String)>` 等

## 根拠

momo の代替実装としての互換性が失われている。同じドキュメントや起動スクリプトで momo と momo-rs を切り替えられるようにする必要がある。

## 参考

- momo-cpp の `--libcamera-control` 実装 (`src/util.cpp`)
- 現状の momo-rs 実装 (`src/main.rs:96-114`)
