# Windows の --debug --package ビルドが Release ディレクトリをハードコードして失敗する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-windows-debug-package
- Polished: 2026-08-20

## 目的

`run.py` の Windows パッケージング処理が `Release/momo.exe` をハードコードしているため、`--debug --package` を指定すると `FileNotFoundError` で失敗する。Windows は multi-config generator のため `--config Debug` の成果物は `Debug/momo.exe` に置かれる。これを修正する。

## 現状

- `run.py` の `_build()` 内の `if args.package:` ブロック (`shutil.copyfile` の Windows 分岐) が `os.path.join(momo_build_dir, "Release", "momo.exe")` をハードコードしている
- ビルド側は `_build()` 内で `configuration` 変数を算出し (`args.debug` / `args.relwithdebinfo` から `Debug` / `Release` / `RelWithDebInfo` を決定)、`cmake --build . --config` に渡している
- CI は `--debug` を指定せず release のみのため未発火の潜在バグ

## 再現手順

- `python3 run.py build windows_x86_64 --debug --package` を実行すると `FileNotFoundError` になる

## 設計方針

- 既存の `configuration` 変数を再利用し、Windows のコピー元パスを `os.path.join(momo_build_dir, configuration, "momo.exe")` にする
- これにより Debug / Release / RelWithDebInfo のどの構成でも正しいディレクトリからコピーする

## 完了条件

- Windows で `--debug --package` が成功する
- Windows で `--relwithdebinfo --package` が成功する
- 通常の `--package` (release) は従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
