# download-binary action が Windows 生成 momo.env の CRLF を処理できず artifact ダウンロードに失敗する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-download-binary-crlf
- Polished: {YYYY-MM-DD}

## 目的

`.github/actions/download-binary/action.yml` が `momo.env` を `source` する際、Windows で生成された `momo.env` は CRLF 改行のため `PACKAGE_NAME` に `\r` が残り、artifact 名不一致で `gh run download` が失敗する。`.github/actions/download/action.yml` には `sed -i 's/\r//g'` があるのにこの action には無い。これを修正する。

## 現状

- `run.py` (726-728 行) がテキストモード `open(..., "w")` で `momo.env` を書き込むため、Windows では CRLF
- `.github/actions/download-binary/action.yml` (132-138 行) が `source momo.env` を実行し、`PACKAGE_NAME` に `\r` が混入
- 同 (142 行) の `gh run download $RUN_ID --name "$PACKAGE_NAME"` が artifact 名不一致で失敗
- `.github/actions/download/action.yml:20` には CRLF 対策の `sed -i 's/\r//g'` がある

## 設計方針

- `source` 前に `sed -i 's/\r//g' momo.env` で CRLF を除去する (download/action.yml と同様)

## 完了条件

- Windows の E2E テスト (有効化時) で artifact ダウンロードが成功する
- 既存プラットフォームの動作が変わらない

## 解決方法

未着手 (PR 作成後に追記する)
