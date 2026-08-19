# run.py の install_sysroot が sysroot 設定変更時に --force を渡せずローカルで詰む

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-sysroot-force-option
- Polished: {YYYY-MM-DD}

## 目的

`sysroot_builder.py` は既存 sysroot が設定と一致しない場合に「`--force` を渡せ」というエラーを出すが、`run.py` の `install_sysroot()` は `--force` を渡す手段が無い。sysroot 設定 JSON を変更した開発者は、`rm -rf` か sysroot_builder.py 直接実行を強いられ、multistrap 時代の rootfs が残っている移行期も同じエラーになる。これを修正する。

## 現状

- `sysroot_builder.py` (614-617 行) が不整合時に `use --force` を要求
- `run.py` の `install_sysroot()` (49-66 行) が `--force` を渡さない
- `run.py` の `install_deps()` (86-92 行) が `install_sysroot()` を呼ぶ

## 設計方針

- `run.py` に `--force-sysroot` 等のオプションを追加し、`install_sysroot()` へ `--force` を伝播する
- または `install_deps` 側で sysroot 設定変更時に自動で再生成する判断を行う

## 完了条件

- ローカルの `run.py build` で sysroot 設定変更後に詰まらず再生成できる
- 既存のビルドフローが変わらない

## 解決方法

未着手 (PR 作成後に追記する)
