# Windows の --debug --package ビルドが Release ディレクトリをハードコードして失敗する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-windows-debug-package
- Polished: {YYYY-MM-DD}

## 目的

`run.py` の Windows パッケージング処理が `Release/momo.exe` をハードコードしているため、`--debug --package` を指定すると `FileNotFoundError` で失敗する。Windows は multi-config generator のため `--config Debug` の成果物は `Debug/momo.exe` に置かれる。これを修正する。

## 現状

- `run.py` の `_package()` (670-674 行) が `os.path.join(momo_build_dir, "Release", "momo.exe")` をハードコード
- `run.py` (651-660 行) は `args.debug` で `cmake --build . --config Debug` を切り替える
- CI は release のみのため未発火の潜在バグ

## 設計方針

- `Configuration` (Debug / Release) をビルド設定から取得してパスに使用する
- `--debug` 指定時は `Debug/momo.exe` からコピーする

## 完了条件

- Windows で `--debug --package` が成功する
- 通常の `--package` (release) は従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
