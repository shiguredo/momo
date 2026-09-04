# CI ワークフローの死設定と意図的無効化を整理する

- Created: 2026-09-02
- Completed: 2026-09-04
- Branch: feature/remove-ci-dead-workflow-settings
- Polished: 2026-09-03
- Milestone: 2026.1.0

## 目的

GitHub Actions に、存在しないワークフローへの参照と、意図を残したままコメントアウトした設定が混在している。死参照は削除する。意図的な一時無効は残骸として消さず、本 issue では再有効化もしない。

C++ の死にコード削除は `0038-remove-dead-code` の担当であり、本 issue ではソースを変更しない。Python の ruff / ty を CI に足す作業は `0039-add-python-static-check-ci` の担当である。DEPS と CI のバージョン一元化は `0041-refactor-deps-single-source` の担当である。

## 現状

`.github/workflows/` にあるワークフローは `build.yml` と `e2e-test.yml` のみである。`claude.yml` はリポジトリに無い。

`build.yml` の `on.push.paths-ignore` に `.github/workflows/claude.yml` がある。マッチしないパスを ignore してもワークフローの起動条件は変わらない。死んだ参照である。

`build.yml` の `create-release` ジョブは、タグが `refs/tags/202*` で始まるとき成果物を `gh release create` する。`.github/actions/download`（`download-artifact`）で `windows_x86_64` / `macos_arm64` / `raspberry-pi-os_armv8` / `ubuntu-22.04_x86_64` / `ubuntu-24.04_x86_64` を取る。`ubuntu-22.04_armv8_jetson` の download だけがコメントアウトされている。一方 `build-ubuntu` の matrix に `ubuntu-22.04_armv8_jetson` は残っており、ビルド自体は走る。CHANGES.md の 2025.1.1 は「NVIDIA Jetson 向けビルドが GitHub Actions で通らないため一時的にコメントアウトする」と記録しているが、現行はビルド matrix が生き、GitHub Release への添付だけが外れた状態である。E2E 用の `.github/actions/download-binary` とは別物である。

`e2e-test.yml` の `on.schedule` はコメントアウトされ、JST 14:00-16:30 に 30 分ごと（平日）回していた、という説明が残っている。E2E は `workflow_dispatch`、`workflow_call`（`build.yml` の `from_build: true`）、および `develop` と `feature/**` への push（`test/**` と `.github/workflows/e2e-test.yml`）では動く。

同じファイルの `e2e_test` matrix に、`windows_x86_64` / `windows-2025` のエントリが `Windows サポート予定` 付きでコメントアウトされている。Windows 向けの momo ビルドジョブ (`build-windows`) は `build.yml` で生きている。

## 設計方針

- 存在しない `claude.yml` の `paths-ignore` エントリは削除する。これが本 issue でワークフローの挙動を変える唯一の差分である
- Jetson の Release download、E2E の `on.schedule`、Windows E2E matrix は再有効化しない。コメントだけ消して無効のままにもしない
- Jetson を GitHub Release に載せる判断、E2E の定期実行を戻す判断、Windows E2E の追加は、いずれも本 issue のカテゴリ `remove` の外である。載せる／戻す／足すときは別 issue にする
- `0038-remove-dead-code` の C++、`0039-add-python-static-check-ci` の `prek.toml` と ruff / ty ステップ、`0041-refactor-deps-single-source` のバージョン参照は触らない

## 完了条件

- `build.yml` の `paths-ignore` から存在しない `claude.yml` が消えている
- `create-release` の `ubuntu-22.04_armv8_jetson` download はコメントアウトのままである
- `e2e-test.yml` の `on.schedule` はコメントアウトのままである（理由コメントも残っている）
- Windows E2E matrix のコメントアウトは残っている
- C++ ソース、`prek.toml`、DEPS のバージョン一元化を本 issue の差分に含めていない

## 解決方法

`build.yml` の `on.push.paths-ignore` から、リポジトリに無い `.github/workflows/claude.yml` を削除した。`create-release` の `ubuntu-22.04_armv8_jetson` download、`e2e-test.yml` の `on.schedule`、Windows E2E matrix のコメントアウトは再有効化せず残した。C++ ソース、`prek.toml`、DEPS は変更していない。CI の Python static check、各プラットフォームのビルド、E2E は成功した。
