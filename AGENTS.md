# AGENTS

- Premature Optimization is the Root of All Evil
- 一切忖度しないこと
- 常に日本語を利用すること
- 全角と半角の間には半角スペースを入れること
- 絵文字を使わないこと
- コメントは全て日本語
- ログメッセージは全て英語
- エラーメッセージは全て英語

## レビューについて

- レビューはかなり厳しくすること
- レビューの表現は、シンプルにすること
- レビューの表現は、日本語で行うこと
- レビューの表現は、指摘内容を明確にすること
- レビューの表現は、指摘内容を具体的にすること
- レビューの表現は、指摘内容を優先順位をつけること
- レビューの表現は、指摘内容を優先順位をつけて、重要なものから順に記載すること
- ドキュメントは別に書いているので、ドキュメトに付いては考慮しないこと
- 変更点とリリースノートの整合性を確認すること

## コミットについて

- 勝手にコミットしないこと
- コミットメッセージは確認すること
- コミットメッセージは日本語で書くこと
- コミットメッセージは命令形で書くこと
- コミットメッセージは〜するという形で書くこと

## サンプルについて

- サンプルは **お手本** なので性能と堅牢性を両立させること

## issues について

- 番号が小さい issues から順番に対応すること
- `{seqnum}-{category}-{short-description}.md` という命名規則を守ること
  - seqnum は `issues/SEQUENCE` ファイルの値を使うこと（9999 を超えたら 5 桁にする）
  - issue を新規作成したら `issues/SEQUENCE` の値を +1 して更新すること
  - 例: `0001-bug-fix-parse-error.md`
  - 例: `0002-fmt-enhance-support-for-joins.md`
- 仕様的に対応が難しい場合は issues/pending/ へ移動すること
- issue を作成したらコミットすること
- 1 issue 完了ごとに 1 コミットすること
- Issue の作成日はファイルのタイトルの後に `Created: YYYY-MM-DD` として記載すること
- Issue の完了日はファイルのタイトルの後に `Completed: YYYY-MM-DD` として記載すること
- Issue を作成した LLM の Model と Version をファイルのタイトルの後に `Model: <model-name> <version>` として記載すること
  - Opus 4.6 や GPT-5.4 など
- Issue はなぜこの対応が必要なのかの根拠を明確にすること

### issue が実は解決してなかった場合

- reopen の理由を issue に書いて issues/closed から issues/ に移動すること (git mv を使うこと)
- reopen の理由は、何がどう解決していなかったのかを明確にすること

### バグが見つかった場合

- issues/ 以下にバグを markdown 形式で登録すること
- バグは再現手順を明確にすること
- できる限りの情報を

### バグを修正した場合

- issues/ 以下のバグを修正した場合は、修正内容を markdown 形式で記載すること
- issues/closed に移動すること (git mv を使うこと)
- issues/closed に移動するときは issue ファイルに「## 解決方法」セクションを追記し、何をどう修正したかを明記すること

### 設計判断が必要な issue の場合

- 外部依存の追加や設計判断が必要で保留中の issue は `issues/pending/` に置くこと
- issues/pending に移動するときは issue ファイルに pending にした理由を明記すること
- pending の issue は修正せずそのまま残す（close しない）

## テストについて

- pbt 以下に unittest を書かないこと
- unittest は pbt で実現できないものだけを書くこと
- 単体テストのファイル名は `tests/test_<module>.rs` とし、`src/<module>.rs` に対応させること
- PBT のファイル名は `pbt/tests/prop_<module>.rs` とし、`src/<module>.rs` に対応させること
- 特定のモジュールに対応しないテストには `test_` や `prop_` プレフィックスを付けないこと
- `#[ignore]` を使わないこと
- テストファイルが長くなった場合はファイル内で `mod` を使って分割すること
  - テストが長くなるのはモジュール自体が大きすぎるサインなので `src/<module>.rs` 側の分割を検討すること
- `src/<module>/` のようにディレクトリモジュールの場合は `pbt/tests/prop_<module>/main.rs` にサブモジュール対応で分割すること

### テストの役割分担

- PBT: 型情報（Strategy）に基づいて入力を生成し、プロパティを検証する（ラウンドトリップ等）
- Fuzzing: 任意入力に対するクラッシュ耐性（パニック安全性）
- 単体テスト: 意図的なエラーパス、境界値など PBT で実現できないケース
- PBT でカバーできるものを単体テストで書かない

### カバレッジ駆動のテスト作成手順

1. 対象モジュールの PBT + 単体テストのカバレッジを llvm-cov で取得する
2. 未カバー行を分類する:
   - 正常系ロジック未カバー → PBT の strategy を修正または PBT を追加する
   - エラーパス未カバー → 単体テストまたは fuzzing で対応する
   - 到達不可能なコード → デッドコードとして削除する
3. PBT に「任意入力でパニックしないことだけを検証するテスト」を書かない（fuzzing の役割）

### カバレッジ取得コマンド例

対象モジュールに関連するテストだけを実行し、カバレッジをマージして確認する:

```bash
# 前回の計測結果をクリアする
cargo llvm-cov clean --workspace
# src/<module>.rs 内の #[cfg(test)] mod tests を実行する
cargo llvm-cov --no-report -p {crate} --lib -- <module>
# tests/test_<module>.rs の単体テストを実行する
cargo llvm-cov --no-report -p {crate} --test test_<module>
# pbt/tests/prop_<module>.rs の PBT を実行する
cargo llvm-cov --no-report -p {crate} --test prop_<module>
# 上記すべての計測結果をマージしてレポートを出力する
cargo llvm-cov report
```

## 変更履歴について

- 変更履歴は `CHANGES.md` に記載すること
- 変更の種別は以下の 4 つを使うこと
  - `[UPDATE]`: 後方互換がある変更
  - `[ADD]`: 後方互換がある追加
  - `[CHANGE]`: 後方互換のない変更
  - `[FIX]`: バグ修正
- エントリは種別の順番を守って記載すること（UPDATE → ADD → CHANGE → FIX の順）
- 機能に直接影響しない変更（ドキュメント追加、リファクタリング等）は `### misc` サブセクションに記載すること
- 未リリースの変更は `## develop` セクションに追記すること
- 各エントリは `- [種別] 変更内容を〜するという形で書く` というフォーマットにすること
- 各エントリの担当者はエントリの次の行に記載し、変更内容より 2 文字分インデントを下げて `- @ユーザー名` の形式にすること
- 担当者の行はそのエントリの最後に書くこと
- 変更内容の説明は日本語で書くこと
- リリース時は `## develop` を `## バージョン` に変更し、`**リリース日**: YYYY-MM-DD` を記載すること

## Rust

- 性能より堅牢性を優先すること
- 依存は最小限にすること
- draft 由来の機能を実装する場合は、根拠資料名、節番号、将来変更される可能性があることをコードコメントで明記すること
- PBT(Property-Based Testing) や Fuzzing でテストを行うこと
- PBT は proptest を使うこと
- Fuzzing は cargo-fuzz を使うこと
- HTTP/1.1 は shiguredo_http11 を使うこと
- TLS は rustls を使う事
- 非同期処理は tokio を使うこと
- ログはできるだけださないが、使う場合は log を使うこと
- 暗号ライブラリは aws-lc-rs を使うこと
