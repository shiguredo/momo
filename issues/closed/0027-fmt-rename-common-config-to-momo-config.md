# CommonConfig を MomoConfig にリネームする

Created: 2026-03-22
Completed: 2026-05-04
Model: Opus 4.6

## 概要

`src/main.rs` の `CommonConfig` 構造体を `MomoConfig` にリネームする。

## 根拠

`CommonConfig` は汎用的すぎる名前であり、momo 固有の設定であることが名前から読み取れない。`MomoConfig` の方がプロジェクトの命名として適切。

## 対象

- `src/main.rs` の `struct CommonConfig` → `struct MomoConfig`
- 同ファイル内の `common` 変数名 → `momo_config` 等への変更も検討

## 解決方法

- `src/main.rs` の `struct CommonConfig` を `struct MomoConfig` にリネーム
- 同ファイル内の変数名 `common` を `momo_config` にリネーム
- ドキュメント (MOMO.md) と他の active な issue (0014, 0026, pending/0009) の `CommonConfig` 参照も `MomoConfig` に更新
- closed 配下の issue ファイルは当時の記録として `CommonConfig` のまま残す
