# SETUP_WINDOWS.md の metadata エスケープ説明を修正する

- Created: 2026-09-15
- Completed: {YYYY-MM-DD}
- Branch: feature/update-setup-windows-metadata-escape
- Polished: {YYYY-MM-DD}

## 目的

`doc/SETUP_WINDOWS.md` の文字列エスケープ手順に従うと `--metadata` が JSON として受理されず、案内どおりに実行できない。Windows のシェル向けに、実際に動く書き方へ直す。

## 現状

- `src/util.cpp` の Sora モード `--metadata` は CLI11 の `is_json` バリデータで `boost::json::parse` し、空でなければ同じ関数で `MomoArgs::sora_metadata` に入れる。プロセスに渡る文字列は JSON 値でなければならない
- `doc/USE_SORA.md` の例は `--metadata '{"access_token": "xyz"}'` であり、キーと値を JSON の二重引用符で書く
- `doc/SETUP_WINDOWS.md` の「PowerShell やコマンドプロンプトで実行する際の注意」は、キーや値を囲む `"` を `\\\"` にせよと書いている。続く PowerShell 例は `--metadata '{\"access_token\": \"xyz\"}'` で、コードブロックの言語は `bash` である
- PowerShell の単引用符は内容をほぼそのまま渡す。例のとおり書くと momo に届くのは `{\"access_token\": \"xyz\"}` であり、JSON ではない。バリデータは `Value ... is not JSON Value` で落ちる

## 設計方針

- 対象は `doc/SETUP_WINDOWS.md` の当該節と実行例に限る
- PowerShell とコマンドプロンプトを分けて書く。両者を同じ `\\\"` 規則にまとめない
- momo に渡す値は `{"access_token": "xyz"}` の形の JSON になるようにする。シェルごとの引用の差だけを説明する
- コマンド例は PowerShell とコマンドプロンプトのそれぞれで実際に起動できることを確認してから載せる
- コードブロックの言語指定をシェルに合わせる

## 完了条件

- 記載どおりのエスケープで `--metadata` が JSON として受理され、momo が起動できる
- PowerShell とコマンドプロンプトの違いが本文から読み取れる
- コードブロックの言語指定が実行するシェルと一致している

## 解決方法

未着手 (PR 作成後に追記する)
