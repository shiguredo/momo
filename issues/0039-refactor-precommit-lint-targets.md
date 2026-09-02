# CI に Python の静的検証 (ruff / ty) がなく、prek.toml の対象から run.py 等が除外されている

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/refactor-precommit-lint-targets
- Polished: {YYYY-MM-DD}
- Milestone: 2026.1.0

## 目的

momo のビルドフロー本体である `run.py` / `canary.py` が `prek.toml` の ruff 対象から除外されており、`buildbase.py` も含めて CI で Python の lint / 型検査が一度も実行されない。正式リリース前に静的検証を CI に組み込み、対象を拡大する。

## 現状

- `prek.toml` (5-6 行) の ruff 対象が `test/` / `tests/` / `sysroot_builder.py` / `jetson_postprocess.py` のみで、`run.py` (764 行) と `canary.py` が対象外
- `prek.toml` (13-19 行) の ty 対象が `test/` のみで、`sysroot_builder.py` / `jetson_postprocess.py` が対象外
- `.github/workflows/build.yml` / `e2e-test.yml` に ruff / ty のステップが無い (Python の静的検証はローカルの prek 依存)
- `buildbase.py` は上流テンプレート由来で、ruff format すると上流と乖離するため対象外とする理由は理解できる

## 設計方針

- `run.py` / `canary.py` を ruff の対象に含める
- `sysroot_builder.py` / `jetson_postprocess.py` を ty の対象に含める
- CI (build.yml) に ruff check / ty check のステップを追加する (sysroot 関連のテスト実行と同様の gate)
- `buildbase.py` は上流同期の観点から、対象に含める場合は上流の lint 設定と整合させる

## 完了条件

- `run.py` / `canary.py` が ruff で検証される
- `sysroot_builder.py` / `jetson_postprocess.py` が ty で検証される
- CI でこれらの静的検証が実行される
- 全ての静的検証がパスする

## 解決方法

未着手 (PR 作成後に追記する)
