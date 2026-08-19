# buildbase.py の get_macos_osver が return を欠いて常に None を返す

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-get-macos-osver
- Polished: {YYYY-MM-DD}

## 目的

`buildbase.py` の `get_macos_osver()` が `platform.mac_ver()[0]` を評価するだけで return していないため、常に `None` を返す。現状は macOS ターゲットが `osver` を参照しないため無影響だが、将来 osver を使う変更を入れた瞬間に誤動作する明らかなバグである。修正する。

## 現状

- `buildbase.py` の `get_macos_osver()` (2202-2203 行) が return を欠如
- `run.py` (475-477 行) の `Platform("macos", get_macos_osver(), ...)` に渡され `osver=None` になる
- テンプレート由来のバグ (上流 melpon/buildbase でも同じ実装)

## 設計方針

- `return platform.mac_ver()[0]` を追加する

## 完了条件

- `get_macos_osver()` が macOS のバージョン文字列を返す
- ビルドフローに影響が無い

## 解決方法

未着手 (PR 作成後に追記する)
