# CI に Python の静的検証 (ruff / ty) がなく、prek.toml の対象から run.py 等が除外されている

- Created: 2026-08-19
- Completed: 2026-09-04
- Branch: feature/add-python-static-check-ci
- Polished: 2026-09-02
- Milestone: 2026.1.0

## 目的

ビルドフロー本体である `run.py` / `canary.py` が `prek.toml` の ruff 対象から外れており、CI にも ruff / ty のステップが無い。ローカルの prek に依存したままでは、対象外のスクリプトと型検査の穴が残る。必要な面に静的検証を載せ、対象を拡大する。

## 現状

`prek.toml` の `ruff-check` / `ruff-format` の `files` は `test/` / `tests/` / `sysroot_builder.py` / `jetson_postprocess.py` に限る。`run.py` と `canary.py` は対象外。`buildbase.py` も対象外である。

`ty-check` は `files = "^test/"`、`pass_filenames = false`、`entry` は `cd test && uv run ty check .` である。`files` はフックを動かすかのフィルタであり、検査パスは常に `test/` 配下の `.` だけである。`sysroot_builder.py` / `jetson_postprocess.py` を `files` に足しても、現行の `entry` のままでは型検査されない。

`.github/workflows/build.yml` と `e2e-test.yml` に ruff / ty のステップは無い。Python の実行検証は、`build.yml` の `Test sysroot Python modules`（`pytest tests/`。`ubuntu-22.04_armv8_jetson` と `raspberry-pi-os_armv8` だけ）と、E2E の `pytest` である。`run.py` 自体は各 matrix のビルドで実行されるが、lint / 型検査は走らない。

`test/pyproject.toml` の `dev` 依存に ruff / ty はある。リポジトリ直下に `pyproject.toml` は無い。closed 0003 / 0004 は直下 `pyproject.toml` で `sysroot_builder.py` / `jetson_postprocess.py` / `tests/` を ty 対象にする作業を「別途推奨」のまま closed にしている。

`buildbase.py` は melpon/buildbase テンプレートで、先頭コメントどおり `curl -LO` で上書きする。ruff format の対象にすると上流差分が消え、次回同期で戻る。

現行の `cd test && uv run ty check .` は診断が多数出る（着手時に 926 件）。これをそのまま CI に載せると既存 E2E 補助コードの型修正が本 issue の本体になる。

`run.py` を `test/` の ruff 設定で `ruff check` すると指摘が出る（着手時に 13 件）。`canary.py` は 0 件だった。

## 設計方針

- `buildbase.py` は ruff / ty の対象外で確定する（0004 のテンプレート方針）。本 issue の「対象拡大」に含めない
- `run.py` / `canary.py` を `ruff-check` の対象に含める。出てくる指摘は本 issue で直すか、直さずに通すルール変更かを PR で明示する。`ruff-format` に含めるかは、format 差分の規模を見て決める
- `sysroot_builder.py` / `jetson_postprocess.py` を ty で検査する。`ty-check` の `files` だけ広げず、`entry` がそれらのパスを実際に検査するように変える。必要なら検査用の設定（直下 `pyproject.toml` の最小追加を含む）も本 issue で入れる
- `test/` 配下の既存 ty 診断は本 issue では一括修正しない。現行の `ty check .`（`test/` 全体）を CI にそのまま載せない
- CI は `build.yml` に載せる。静的検証の job / step に、sysroot pytest と同じ `if`（Jetson / Raspberry Pi OS だけ）を付けない。`run.py` を使う全構成のゲートになる場所に置く。許可済みなら既存の `astral-sh/setup-uv` を使う。新規に許可リスト外の action は足さない
- `e2e-test.yml` には ruff / ty を足さない（`test/` 全体の ty をゲートにしない判断と揃える）
- `0056-remove-ci-dead-workflow-settings` が扱う `claude.yml` の paths-ignore、Jetson の Release download、E2E の schedule / Windows matrix は触らない
- `0041-refactor-deps-single-source` の DEPS 一元化と差分を混ぜない

## 完了条件

- `run.py` / `canary.py` が `ruff-check` の対象であり、その検証がパスする
- `sysroot_builder.py` / `jetson_postprocess.py` が ty で実際に検査され、その検証がパスする
- `build.yml` で上記の静的検証が、sysroot pytest 限定の `if` に閉じずに実行される
- `buildbase.py` を ruff / ty の対象に含めていない
- `test/` 全体の ty を CI の必須ゲートにしていない
- `e2e-test.yml` と 0056 の対象コメントを本 issue の差分に含めていない

## 解決方法

`prek.toml` の `ruff-check` / `ruff-format` に `run.py` / `canary.py` を含めた。format 差分は無かったので format 対象にも入れた。既存の `test/` 向け `ty-check` は残し、`ty-check-sysroot` を追加して `uv run --with ty --python 3.13 ty check sysroot_builder.py jetson_postprocess.py` で検査するようにした。直下 `pyproject.toml` は入れてない。`build.yml` に独立 job `python-static-check` (`ubuntu-slim`) を置き、`uvx ruff@0.14.14` と上記 ty を sysroot pytest と同じ matrix `if` なしで実行する。`e2e_test` / `create-release` / `notification` の `needs` に足した。`run.py` の ruff 指摘はルール緩和せず直した（型注釈の更新、`RunError` への置き換え、モジュール logger、ネストした `if` / `with` の結合）。`canary.py` は指摘が無く未変更。`buildbase.py` と `e2e-test.yml`、存在しない `claude.yml` の `paths-ignore` や Jetson Release / E2E schedule / Windows matrix のコメントアウトは触っていない。CI の Python static check、各プラットフォームのビルド、E2E は成功した。
