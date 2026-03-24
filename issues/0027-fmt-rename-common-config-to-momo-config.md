# CommonConfig を MomoConfig にリネームする

Created: 2026-03-22
Model: Opus 4.6

## 概要

`src/main.rs` の `CommonConfig` 構造体を `MomoConfig` にリネームする。

## 根拠

`CommonConfig` は汎用的すぎる名前であり、momo 固有の設定であることが名前から読み取れない。`MomoConfig` の方がプロジェクトの命名として適切。

## 対象

- `src/main.rs` の `struct CommonConfig` → `struct MomoConfig`
- 同ファイル内の `common` 変数名 → `momo_config` 等への変更も検討
