# sysroot builder の再利用判定が suite 内のパッケージ更新を拾わず古い rootfs を使い続ける

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-sysroot-fingerprint-stale
- Polished: 2026-09-13

## 目的

`sysroot_builder.py` の再利用判定 (fingerprint) に「実際に解決されたパッケージバージョン」が含まれていない。suite 名 (trixie / jammy / r36.3) が同じ限り、設定した APT リポジトリ内でパッケージが更新 (セキュリティ修正などによるバージョン変更) されても拾わず、ローカルビルドは古い rootfs を永続的に再利用する。CI は毎回新規生成なので常に最新だが、ローカルビルドは無期限に古いままになる。これを修正する。

## 現状

- `sysroot_builder.py` の `sysroot_config_fingerprint()` が `name` / `arch` / `triplet` / `packages` / `repositories` (url / suite / components / signed_by の SHA-256 / pin_priority) のみからハッシュを計算し、解決されたバージョンを含まない
- `build_sysroot()` の再利用判定は、manifest の `format_version` と `fingerprint` のみを参照し、`deb_files` は使わない
- `build_sysroot()` が manifest へ書き込む `deb_files` は deb ファイル名の一覧 (`{package}_{version}_{arch}.deb` の形式でバージョンを含む) だが、再利用判定には使われない
- `run.py` の `install_sysroot()` は毎回ビルドせず、`build_sysroot()` の再利用判定に従う

## 設計方針

- 再利用判定を「設定由来の fingerprint と APT 側の解決結果が両方一致したときだけ再利用」に変更する。解決結果は、`deb_files` から得られるバージョン一覧、または apt のパッケージリストから計算したハッシュで表現し、manifest へ fingerprint とは別フィールドとして記録する
- パッケージ更新により解決結果だけが変わった場合は、既存 sysroot が同一 builder の生成物 (format_version が一致する manifest を持つ) なら、`--force` を要求せず `build_sysroot()` 内で自動的に再生成して置き換える。`run.py` の `--force` 伝播 ([0034](./0034-fix-sysroot-force-option.md)) に依存しない
- 設定 JSON 自体の変更 (設定由来 fingerprint の不一致)、`format_version` の不一致、manifest を持たない既存ディレクトリは、従来通り「use `--force`」エラーとする
- manifest の形式が変わるため `MANIFEST_VERSION` を上げる
- ローカルで suite 更新を拾えるようにする

## 完了条件

- ローカルビルドでも、設定した APT リポジトリ内のパッケージ更新が反映される (自動で再生成される)
- 設定も APT 側の解決結果も変わらない場合は再利用される (`.deb` のダウンロードと展開は発生しない。更新検知のための apt のパッケージリスト取得は毎回行う)
- `tests/sysroot_builder/test_sysroot_builder.py` のテストが更新される (fingerprint に解決結果が含まれること、解決結果の変化で自動再生成されること)

## 解決方法

未着手 (PR 作成後に追記する)
