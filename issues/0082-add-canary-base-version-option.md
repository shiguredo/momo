# canary.py に --release / --hotfix オプションを追加し develop 限定で実行する

- Created: 2026-10-27
- Completed: {YYYY-MM-DD}
- Branch: feature/add-canary-base-version-option
- Polished: {YYYY-MM-DD}

## 目的

`canary.py` は VERSION のベースバージョンを据え置いたまま canary 番号を進めるだけなので、マイナー up (`2025.1.0` → `2025.2.0-canary.0`) やパッチ up (`2026.1.0` → `2026.1.1-canary.0`) を始めるときは、実行者が VERSION を手で書き換えてから実行している。手作業には次の危険がある。

- ベースバージョンの書き換えミス (桁や区切りの取り違え) が、そのままタグ作成と `git push` まで進む
- 基点の切り替えなのか canary 番号の単純な進行なのかが、実行コマンドから読み取れず、`git log` にも残らない
- canary リリースは `develop` からのみ実施する運用だが、別ブランチで誤って実行しても止まらない。`--dry-run` でも「想定通りのコマンド表示」が出るため、ブランチ違いに気付かず本実行してしまう

基点を進める操作をオプションとして明示させ、誤操作を実行前に止められるようにする。

## 現状

- `canary.py` の `update_sdk_version` は `VERSION` の内容に `re.match(r"(\d{4}\.\d+\.\d+)(-canary\.(\d+))?")` を掛け、`-canary.N` が無ければ `-canary.0` を付与し、有れば `N` を 1 増やすだけ。ベースバージョン (`major.minor.patch`) は一切変更しない
- `main` は argparse で `--dry-run` だけを定義しており、ブランチを検証していない
- `git_operations` は `git commit -am`、`git tag`、`git push`、`git push origin <tag>` の順に `check=True` で実行する
- コミットメッセージは `[canary] Update VERSION to {new_version}` 固定

## 設計方針

- `--release` と `--hotfix` を argparse の排他グループ (`mutually_exclusive_group`) として追加する。両方同時指定は argparse の usage 表示で終了する
- VERSION が安定版 (`-canary.N` 無し) のときの挙動
  - `--release`: マイナーを 1 増やしパッチを 0 にした `major.(minor+1).0-canary.0` を作る
  - `--hotfix`: パッチを 1 増やした `major.minor.(patch+1)-canary.0` を作る
  - どちらも指定しない: usage を表示して終了コード 0 で終了する (何も変更しない)
- VERSION が既に `-canary.N` のときの挙動
  - どちらも指定しない: 現行どおり `-canary.(N+1)` にする
  - `--release` または `--hotfix` を指定した場合: 基点を二重に進める事故を防ぐためエラーメッセージと usage を表示して非 0 で終了する
- 実行ブランチは `git rev-parse --abbrev-ref HEAD` による `develop` の完全一致のみ受け付ける。Git Flow 運用のため `develop` で始まる作業ブランチは作らない前提であり、前方一致やエスケープ手段 (`--force` 等) は設けない。`--dry-run` でも同じ検証を行い、`develop` 以外では実行しない
- エラー時 (`develop` 以外、`-canary.N` 無しでオプション未指定) はエラーメッセージと usage を表示する。usage を出すためのエラー経路は argparse の `parser.error` を使わずに独自に扱う。ただし排他グループの競合だけは argparse の標準動作に委ねる
- 変更対象ファイルは現行実装どおり `VERSION` のみとする。カレンダー年の変更は canary 実行者が VERSION を直接編集して行うものとし、`canary.py` では年を変更しない
- コミットメッセージ、`CHANGES.md` への記録、git 操作の順序と失敗時の扱い (既存タグとの衝突を含む)、バージョン文字列の検証 (桁数・桁あふれ・フォーマット崩れ)、出力方法 (`print` のまま) は現行実装から変えない

## 完了条件

- `develop` 以外のブランチで `canary.py` を実行すると、`--dry-run` も含めてエラーメッセージと usage が表示され、VERSION が変更されない
- VERSION が `2026.1.0` のとき、`canary.py --release` で `2026.2.0-canary.0`、`canary.py --hotfix` で `2026.1.1-canary.0` になり、従来のタグ作成と push まで実行される
- VERSION が `2026.1.0` でオプションを指定しなければ usage が表示され、終了コード 0 で何も変更されない
- VERSION が `2026.1.0-canary.0` のとき、オプション無しで `2026.1.0-canary.1` になり、`--release` / `--hotfix` のいずれかを指定するとエラーメッセージと usage が表示され、VERSION が変更されない
- `--release` と `--hotfix` を同時に指定すると usage が表示され、VERSION が変更されない

## 解決方法

実装時に追記する。
