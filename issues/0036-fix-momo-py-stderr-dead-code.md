# test/momo.py の stderr 読み取りデッドコードを削除する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-momo-py-stderr-dead-code
- Polished: 2026-09-13

## 目的

`test/momo.py` の `Momo` クラスは `subprocess.Popen(cmd, stdout=None, stderr=None, text=True)` で momo を起動するため `self.process.stderr` は常に `None` である。にもかかわらず「stderr を表示してデバッグ」するコードが 3 ブロックあり、全て恒常的にスキップされるデッドコードになっている。コードを見た実装者が「起動失敗時に stderr を自前で表示できる仕組みがある」と誤解する原因になるため、デッドコードを削除して現行設計に合わせる。

## 現状

- `test/momo.py` の `__enter__` の `subprocess.Popen` は `stdout=None` / `stderr=None` を指定しており、momo の出力は親プロセスへ継承される
- `_wait_for_startup` 内に `self.process.stderr` を参照する 3 ブロックがある（それぞれ プロセス早期終了時の読み出し、起動待機中 5 秒ごとの `select` / `fcntl` による非ブロッキング読み出し、起動タイムアウト時の読み出し）
- `stderr=None` のとき `self.process.stderr` は常に `None` のため、いずれのブロックも実行されない
- なお継承された momo の stdout / stderr は、pytest の既定のキャプチャではテスト失敗時のレポート（Captured stdout / Captured stderr）に含まれ、CI ログから確認できる

## 設計方針

- `subprocess.Popen` は `stdout=None` / `stderr=None` のまま変更しない
  - 出力を親プロセスへ継承して CI ログで確認できる方式は 2026-06-16 の「E2E テストの momo プロセス出力を CI ログに表示する」で意図的に導入され、`CHANGES.md` の develop 節にも記載されている
  - open issue `0002-fix-e2e-test-transient-failure` の再発時対応手順も、「momo の stdout / stderr は CI ログに表示される」ことを前提にしている
- `stderr=PIPE` による自前収集へは変更しない
  - 起動完了後は stderr を読み出す主体がなくなり、ログ量によってはパイプバッファが満杯になって momo の書き込みがブロックするリスクがある
  - 起動失敗時でも継承された stderr は pytest の失敗レポートで確認できるため、収集の利点がない
- `_wait_for_startup` 内の 3 ブロックと、ブロック内だけで使われる `select` / `fcntl` / `os` のローカル import を削除する

## 完了条件

- `test/momo.py` に `self.process.stderr` への参照が残っていない
- `_wait_for_startup` のエラー処理（`RuntimeError` の送出）が損なわれない
- 既存の E2E テスト（pytest）が通る

## 解決方法

未着手 (PR 作成後に追記する)
