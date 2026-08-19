# test/momo.py の stderr 読み取りコードがデッドコードで起動失敗の原因が調査不能

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-momo-py-stderr-dead-code
- Polished: {YYYY-MM-DD}

## 目的

`test/momo.py` の `Momo` ラッパーは `subprocess.Popen(cmd, stdout=None, stderr=None, text=True)` で momo を起動するため `self.process.stderr` は常に `None` である。にもかかわらず「stderr を表示してデバッグ」するコードが 3 箇所あり、全て恒常的にスキップされるデッドコードになっている。momo が起動失敗した原因 (引数ミス等) がテスト出力から一切読み取れず、原因調査が著しく困難。これを修正する。

## 現状

- `test/momo.py` (327-332, 688-691, 730-749, 763-767 行) に `self.process.stderr` を読むデッドコードが複数
- `subprocess.Popen(..., stderr=None)` のため `self.process.stderr` は常に `None`
- 起動失敗時の stderr ログが取得できない

## 設計方針

- `subprocess.Popen` で `stderr=PIPE` を指定し、momo の stderr を収集できるようにする
- 起動失敗時に stderr 内容をテストログへ出力する
- デッドコードを削除する

## 完了条件

- momo 起動失敗時に stderr ログがテスト出力から確認できる
- デッドコードが削除されている

## 解決方法

未着手 (PR 作成後に追記する)
