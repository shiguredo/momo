# canary.py に --release / --hotfix オプションを追加し develop 限定で実行する

- Created: 2026-10-27
- Completed: {YYYY-MM-DD}
- Branch: feature/add-canary-base-version-option
- Polished: 2026-10-27

## 目的

`canary.py` は VERSION のベースバージョンを据え置いたまま canary 番号を進めるだけなので、マイナー up (`2025.1.0` → `2025.2.0-canary.0`) やパッチ up (`2026.1.0` → `2026.1.1-canary.0`) を始めるときは、実行者が VERSION を手で書き換えてから実行している。手作業には次の危険がある。

- ベースバージョンの書き換えミス (桁や区切りの取り違え) が、そのままタグ作成と `git push` まで進む
- 基点の切り替えなのか canary 番号の単純な進行なのかが、実行コマンドから読み取れず、`git log` にも残らない
- canary リリースは `develop` からのみ実施する運用だが、別ブランチで誤って実行しても止まらない。`--dry-run` でも「想定通りのコマンド表示」が出るため、ブランチ違いに気付かず本実行してしまう

基点を進める操作をオプションとして明示させ、誤操作を実行前に止められるようにする。

## 現状

- `canary.py` の `update_sdk_version` は `VERSION` の内容に `re.match(r"(\d{4}\.\d+\.\d+)(-canary\.(\d+))?")` を掛け、`-canary.N` が無ければ `-canary.0` を付与し、有れば `N` を 1 増やすだけ。ベースバージョン (`major.minor.patch`) は一切変更しない
- `main` は argparse で `--dry-run` だけを定義し、引数を解析した後で `VERSION` を読み込み、`update_sdk_version`、`write_version_file`、`git_operations` の順に呼ぶ。ブランチの検証は行わない
- `update_sdk_version` はベースバージョンの算出と canary 番号の採番を 1 関数に閉じており、`-canary.N` の有無もこの関数の中で判定している
- `git_operations` は `git commit -am`、`git tag`、`git push`、`git push origin <tag>` の順に `check=True` で実行する
- コミットメッセージは `[canary] Update VERSION to {new_version}` 固定

## 設計方針

- `--release` と `--hotfix` を argparse の排他グループ (`mutually_exclusive_group`) として追加する。両方同時指定は argparse の標準動作に委ね、usage とエラーメッセージを表示して終了コード 2 で終了する
- 実行ブランチの検証は `main` の引数解析直後、`VERSION` の読み込みより前に行う。`git rev-parse --abbrev-ref HEAD` の結果が `develop` と完全一致しない場合は、VERSION を書き換えず git 操作も行わずに終了する。Git Flow 運用のため `develop` で始まる作業ブランチは作らない前提であり、前方一致やエスケープ手段 (`--force` 等) は設けない。`--dry-run` でも同じ検証を行い、`develop` 以外では何も表示せずに通過させることはしない
- ブランチ検証の結果は、後述する VERSION の状態やオプションの指定より優先する。`develop` 以外での実行は、VERSION の状態にかかわらず必ずエラー終了となる
- VERSION が安定版 (`-canary.N` 無し) のときの挙動
  - `--release`: マイナーを 1 増やしパッチを 0 にした `<4 桁の年>.(<マイナー>+1).0-canary.0` を作る (例: `2026.1.0` → `2026.2.0-canary.0`)
  - `--hotfix`: パッチを 1 増やした `<4 桁の年>.<マイナー>.(<パッチ>+1)-canary.0` を作る (例: `2026.1.0` → `2026.1.1-canary.0`)
  - どちらも指定しない: 何も変更せず usage を表示して終了コード 0 で終了する
- VERSION が既に `-canary.N` のときの挙動
  - どちらも指定しない: 現行どおり `-canary.(N+1)` にする
  - `--release` または `--hotfix` を指定した場合: 基点を二重に進める事故を防ぐため、VERSION を書き換えずにエラーメッセージと usage を表示して終了する
- 年は式でも据え置く。`canary.py` は 4 桁の年を変更しない
- 分岐の置き場所を次のとおり確定する
  - `update_sdk_version` に基点の方針 (現行どおり維持 / マイナー up / パッチ up) を渡す形にシグネチャを変更し、`-canary.N` の有無の判定、ベースバージョンの算出、canary 番号の採番はこの関数に集約する
  - `-canary.N` 付き VERSION に `--release` / `--hotfix` が指定された場合のエラーは、`main` が `VERSION` の内容と解析済みの引数から判断し、`update_sdk_version` を呼ぶ前に終了する
  - `write_version_file` と `git_operations` は変更せず、呼び出し順も変えない。`VERSION` の末尾改行の有無 (`write_version_file` が改行を書かない現行挙動) も変えない
- usage とエラーメッセージの出力先は次のとおり確定する
  - `develop` 以外での実行: エラーメッセージと usage を stderr に表示して終了コード 1 で終了する
  - `-canary.N` 付き VERSION に `--release` / `--hotfix` を指定: エラーメッセージと usage を stderr に表示して終了コード 1 で終了する
  - 安定版 VERSION でオプション未指定: usage を stderr に表示して終了コード 0 で終了する
  - `--release` と `--hotfix` の同時指定: argparse の標準動作により usage とエラーメッセージの両方を stderr に表示して終了コード 2 で終了する
  - 上記以外の usage 表示とエラーメッセージは argparse の `parser.error` を使わずに独自に扱う
- 変更対象ファイルは現行実装どおり `VERSION` のみとする。`canary.py` は `CHANGES.md` を変更しない。カレンダー年の変更は canary 実行者が `VERSION` を直接編集して行うものとし、年をまたいでマイナー up したい場合は実行者が `VERSION` の年とベースバージョンを書き換えてから実行する
- コミットメッセージ、git 操作の順序と失敗時の扱い (既存タグとの衝突を含む)、バージョン文字列の検証 (桁数・桁あふれ・フォーマット崩れ)、`--dry-run` の表示内容、出力方法 (`print` のまま) は現行実装から変えない

## 完了条件

- `develop` 以外のブランチで `canary.py` を実行すると、`--dry-run` の有無や VERSION の状態にかかわらず、エラーメッセージと usage が stderr に表示され、終了コード 1 で終了し、VERSION が変更されない
- VERSION が `2026.1.0` のとき、`canary.py --release` で `2026.2.0-canary.0`、`canary.py --hotfix` で `2026.1.1-canary.0` になり、従来のタグ作成と push まで実行される
- VERSION が `2026.1.0` でオプションを指定しなければ usage が stderr に表示され、終了コード 0 で終了し、VERSION が変更されない
- VERSION が `2026.1.0-canary.0` と `2026.1.0-canary.9` のとき、オプション無しでそれぞれ `2026.1.0-canary.1` と `2026.1.0-canary.10` になり、従来のタグ作成と push まで実行される
- VERSION が `2026.1.0-canary.0` のとき、`--release` / `--hotfix` のいずれかを指定するとエラーメッセージと usage が stderr に表示され、終了コード 1 で終了し、VERSION が変更されない
- `--release` と `--hotfix` を同時に指定すると usage とエラーメッセージが stderr に表示され、終了コード 2 で終了し、VERSION が変更されない
- `CHANGES.md` と `VERSION` 以外のファイルが変更されていない

## 解決方法

実装時に追記する。
