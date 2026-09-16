# download-binary action が Windows 生成 momo.env の CRLF を処理できず artifact ダウンロードに失敗する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-download-binary-crlf
- Polished: 2026-09-13

## 目的

`.github/actions/download-binary/action.yml` が `momo.env` を `source` する際、Windows で生成された `momo.env` は CRLF 改行のため `PACKAGE_NAME` に `\r` が残り、artifact 名不一致でダウンロードが失敗する。`.github/actions/download/action.yml` には `sed -i 's/\r//g'` があるのにこの action には無い。これを修正する。

## 現状

- `run.py` の `_build` 内 (build コマンドの `--package` 処理) がテキストモード `open(..., "w")` で `momo.env` を書き込むため、Windows では CRLF
- `.github/actions/download-binary/action.yml` では `source momo.env` が 2 箇所ある:
  - 「Get package name from current workflow run」ステップ (`current_artifact` 時): `source momo.env` 後に `PACKAGE_NAME` を出力
  - 「Download artifacts from previous workflow run」ステップ (`previous_artifact` 時): `source momo.env` 後に `gh run download $RUN_ID --name "$PACKAGE_NAME"` を実行
- Windows で生成された `momo.env` を `source` すると `PACKAGE_NAME` に `\r` が混入し、artifact 名不一致で失敗する
- `.github/actions/download/action.yml` の「Env to output」ステップには CRLF 対策の `sed -i 's/\r//g'` がある

## 設計方針

- `source` 前に `sed -i 's/\r//g' momo.env` で CRLF を除去する (download/action.yml と同様)
- 上記の 2 箇所、すなわち「Get package name from current workflow run」ステップと「Download artifacts from previous workflow run」ステップの両方に適用する

## 完了条件

- Windows のビルド成果物を使う E2E テスト (Windows matrix 有効化時) で、`current_artifact` / `previous_artifact` のどちらの経路でも artifact ダウンロードが成功する
- 既存プラットフォームの動作が変わらない

## 解決方法

未着手 (PR 作成後に追記する)
