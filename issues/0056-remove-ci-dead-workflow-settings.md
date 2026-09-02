# CI ワークフローの死設定と意図的無効化を整理する

- Created: 2026-09-02
- Completed: {YYYY-MM-DD}
- Branch: feature/remove-ci-dead-workflow-settings
- Polished: {YYYY-MM-DD}
- Milestone: 2026.1.0

## 目的

GitHub Actions に、存在しないワークフローへの参照と、意図を残したままコメントアウトした設定が混在している。死設定は削除し、一時無効は「残骸」として消さず、残すか再有効化するかを決めてから触る。

C++ の死にコード削除は `0038-remove-dead-code` の担当であり、本 issue ではソースを変更しない。Python の ruff / ty を CI に足す作業は `0039-add-python-static-check-ci` の担当である。

## 現状

`.github/workflows/` にあるワークフローは `build.yml` と `e2e-test.yml` のみである。`claude.yml` はリポジトリに無い。

`build.yml` の `on.push.paths-ignore` に `.github/workflows/claude.yml` がある。存在しないファイルを ignore してもビルドは動かないが、死んだ参照である。

`build.yml` の `create-release` ジョブは、タグ `refs/tags/202*` のとき成果物を `gh release create` する。`windows_x86_64` / `macos_arm64` / `raspberry-pi-os_armv8` / `ubuntu-22.04_x86_64` / `ubuntu-24.04_x86_64` の download は有効。`ubuntu-22.04_armv8_jetson` の download だけがコメントアウトされている。一方 `build-ubuntu` の matrix に `ubuntu-22.04_armv8_jetson` は残っており、ビルド自体は走る。CHANGES.md の 2025.1.1 は「NVIDIA Jetson 向けビルドが GitHub Actions で通らないため一時的にコメントアウトする」と記録しているが、現行はビルド matrix が生き、Release への添付だけが外れた状態である。

`e2e-test.yml` の `on.schedule` はコメントアウトされ、JST 14:00-16:30 に 30 分ごと（平日）回していた、という説明が残っている。E2E は `workflow_dispatch` / `workflow_call`（`build.yml` の `from_build: true`）/ `develop` と `feature/**` への `test/**` の push では動く。

同じファイルの `e2e_test` matrix に、`windows_x86_64` / `windows-2025` のエントリが `Windows サポート予定` 付きでコメントアウトされている。Windows 向けの momo ビルドジョブ (`build-windows`) は `build.yml` で生きている。

## 設計方針

- 存在しない `claude.yml` の `paths-ignore` エントリは削除する
- Jetson の Release download と E2E の schedule / Windows matrix は、コメントを外すか残すかを着手時に決める。コメントだけ消して無効のままにしない（無効化の理由が消える）
- Jetson を GitHub Release に載せる場合は download のコメントを外す。ビルド matrix は既に生きているので、Release 成果物の構成が変わる。載せない場合はコメントアウトを残す
- E2E の schedule を戻す場合は cron を有効化する。戻さない場合は理由コメントを残す
- Windows の E2E matrix を有効化することは、ランナーとテストの追加であり本 issue ではやらない。コメントは残す
- `0038-remove-dead-code` と `0039-add-python-static-check-ci` の対象ファイルに、本 issue 以外の目的の差分を混ぜない

## 完了条件

- `build.yml` の `paths-ignore` から存在しない `claude.yml` が消えている
- Jetson の Release download について、載せるか載せないかが本文の方針どおりワークフローに反映されている（載せるなら download が有効、載せないならコメントアウトが残っている）
- E2E の schedule について、再有効化するか無効のまま理由コメントを残すかが反映されている
- Windows E2E matrix のコメントアウトは残っている
- C++ ソースと `prek.toml` を本 issue の差分に含めていない

## 解決方法

未着手 (PR 作成後に追記する)
