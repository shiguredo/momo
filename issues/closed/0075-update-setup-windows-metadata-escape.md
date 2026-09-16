# SETUP_WINDOWS.md の metadata エスケープ説明を修正する

- Created: 2026-09-15
- Completed: 2026-09-15
- Branch: feature/update-setup-windows-metadata-escape
- Polished: 2026-09-15

## 目的

`doc/SETUP_WINDOWS.md` の文字列エスケープ手順に従うと `--metadata` が JSON として受理されず、案内どおりに実行できない。Windows のシェル向けに、実際に動く書き方へ直す。

## 現状

- `src/util.cpp` の `Util::ParseArgs` で、Sora モード `--metadata` はローカルの `CLI::Validator` (`is_json`) が `boost::json::parse` する。空でなければ同じ関数で `MomoArgs::sora_metadata` に入れる。バリデータに渡る文字列は JSON 値でなければならない。失敗時の文言は `Value ` + 入力 + ` is not JSON Value` である
- `doc/USE_SORA.md` の例は `--metadata '{"access_token": "xyz"}'` であり、キーと値を JSON の二重引用符で書く
- `doc/SETUP_WINDOWS.md` の「PowerShell やコマンドプロンプトで実行する際の注意」は、キーや値を囲む `"` を `\\\"` にせよと書いている。続く PowerShell 例は `--metadata '{\"access_token\": \"xyz\"}'` で、コードブロックの言語は `bash` である
- PowerShell の単引用符では、例の内側は `{\"access_token\": \"xyz\"}` になる。これは JSON ではない。この文字列が `is_json` に入ればバリデータは失敗する。ネイティブ exe の `argv` に何が届くかは、実装時に PowerShell とコマンドプロンプトのそれぞれで確認する（現状の注意文は両シェルに同じ `\\\"` を課している）

## 設計方針

- 対象は `doc/SETUP_WINDOWS.md` の当該節と実行例に限る
- PowerShell とコマンドプロンプトを分けて書く。見出し・本文・例のいずれでも、両シェルに同じ `\\\"` 規則を残さない
- momo に渡す値は `{"access_token": "xyz"}` の形の JSON になるようにする。シェルごとの引用の差だけを説明する。行継続は PowerShell では `` ` ``、コマンドプロンプトでは `^` であり、引用規則と一緒にシェルごとの例へ分ける
- PowerShell で引用が要る理由は、`{` がスクリプトブロックとして解釈されることである。`"` を `\\\"` にする話ではない
- コマンド例は、Windows 上の PowerShell とコマンドプロンプトで `--metadata` が `is_json` を通過することを確認してから載せる。観察するのは CLI が JSON 不正で終了しないことであり、Sora Labo への接続成功までは求めない。Windows が無い場合は推測で例を書かない
- コードブロックの言語指定を、その例を実行するシェルに合わせる

## 完了条件

- 記載どおりの書き方で `--metadata` が `is_json` を通過する（`is not JSON Value` で終了しない）
- PowerShell とコマンドプロンプトの引用の違いが本文から読み取れる。両シェルに同じ `\\\"` 規則が残っていない
- コードブロックの言語指定が、その例を実行するシェルと一致している
- 例は Windows 上の PowerShell とコマンドプロンプトで、上記の CLI 通過を確認したものである

## 解決方法

2026-09-15 に `doc/SETUP_WINDOWS.md` の `--metadata` 案内を、PowerShell とコマンドプロンプトで分けて書き直した。

- PowerShell は JSON 全体を単引用符で囲み、内側の `"` を `\"` と書く。`{` がスクリプトブロックにならないようにするためである。コードブロックの言語は `powershell` にした
- コマンドプロンプトは JSON 全体を二重引用符で囲み、内側の `"` を `\"` と書く。行継続は `^` である。コードブロックの言語は `batch` にした
- 実行例の `--signaling-urls` は `wss://sora.sora-labo.shiguredo.app/signaling` にした
- Windows 上で、PowerShell の単引用符形式と `` ` `` による複数行、コマンドプロンプトの二重引用符形式と `^` による複数行を確認した。`--metadata` は `is_json` を通過した

