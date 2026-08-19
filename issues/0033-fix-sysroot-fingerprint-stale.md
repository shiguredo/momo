# sysroot builder の再利用判定が suite 内のパッケージ更新を拾わず古い rootfs を使い続ける

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-sysroot-fingerprint-stale
- Polished: {YYYY-MM-DD}

## 目的

`sysroot_builder.py` の再利用判定 (fingerprint) に「実際に解決されたパッケージバージョン」が含まれていない。suite 名 (trixie / jammy / r36.3) が同じ限り、apt リポジトリ内のパッケージ更新 (セキュリティ修正を含む) を拾わず、ローカルビルドは古い rootfs を永続的に再利用する。CI は毎回新規生成なので常に最新だが、ローカルビルドは無期限に古いままになる。これを修正する。

## 現状

- `sysroot_builder.py` の fingerprint 生成 (293-324 行) が `name` / `arch` / `triplet` / `packages` / `repositories` のみで、解決されたバージョンを含まない
- 再利用判定 (606-611 行) は fingerprint のみを参照
- manifest の `deb_files` (677 行) には解決済みバージョンが記録されるが判定に使われない
- `run.py` の `install_sysroot()` は毎回ビルドせず、再利用判定に従う

## 設計方針

- fingerprint に、解決されたパッケージのバージョン一覧 (または apt のパッケージリストのハッシュ) を含める
- または再利用判定を、manifest に記録した `deb_files` のバージョンと現在の解決結果の比較に変える
- ローカルで suite 更新を拾えるようにする

## 完了条件

- ローカルビルドでも、リポジトリ内のパッケージ更新が反映される
- 設定が変わらない場合は従来通り再利用される (ビルド時間が増えない)
- テストが更新される

## 解決方法

未着手 (PR 作成後に追記する)
