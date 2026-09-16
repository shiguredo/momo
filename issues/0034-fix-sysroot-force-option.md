# run.py の install_sysroot が sysroot 設定変更時に --force を渡せずローカルで詰む

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-sysroot-force-option
- Polished: 2026-09-13

## 目的

`sysroot_builder.py` は既存 sysroot が設定と一致しない場合に「`use --force`」というエラーを出すが、`run.py` の `install_sysroot()` は `--force` を渡す手段が無い。sysroot 設定 JSON を変更した開発者は、`rm -rf` か sysroot_builder.py 直接実行を強いられる。これを修正する。

## 現状

- `sysroot_builder.py` の `build_sysroot()` が不整合時に `use --force` を要求する
- `run.py` の `install_sysroot()` が `--force` を渡さない
- `run.py` の `install_deps()` 内の sysroot 分岐が `install_sysroot()` を呼ぶ

## 設計方針

- `run.py` の `build` サブコマンドに `--force-sysroot` を追加し、`install_sysroot()` へ `force` パラメータとして伝播して、指定時だけ subprocess へ `--force` を渡す
- オプション未指定時は現行どおり `--force` を渡さず、デフォルトの再利用判定を変えない
- 自動再生成 (`install_deps` 側での判断) は採用しない。理由は次のとおり
  - `install_sysroot()` は canonical (`sysroot_builder.py`) に実装を閉じ込めてサブプロセスとして起動する設計であり、manifest 照合の再実装は重複になる
  - builder は「設定由来か不明な既存ディレクトリを黙って削除または再利用しない」方針 (`tests/sysroot_builder/test_sysroot_builder.py` の `test_build_sysroot_rejects_stale_directory_without_force` 等) で、再生成には利用者の明示を求める
  - パッケージ更新による解決結果の変化は [0033](./0033-fix-sysroot-fingerprint-stale.md) が `build_sysroot()` 内で自動再生成として扱う範囲であり、設定由来 fingerprint の不一致は従来通り `use --force` とする。本 issue は設定変更時に `--force` を渡す手段の提供に限定する

## 完了条件

- `--force-sysroot` 指定時の `run.py build <target>` で、sysroot 設定 JSON 変更後に古い rootfs が置き換わり再生成される
- 未指定時は現行どおり (一致する manifest があれば再利用、不整合なら `use --force` エラー) であり、既存のビルドフローが変わらない

## 解決方法

未着手 (PR 作成後に追記する)
