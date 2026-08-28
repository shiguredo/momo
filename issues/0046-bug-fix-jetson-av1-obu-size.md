# Jetson エンコーダが AV1 の OBU サイズを 1 バイト固定で読みバッファを越境する

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-jetson-av1-obu-size
- Polished: {YYYY-MM-DD}

## 目的

Jetson の AV1 エンコード出力から `OBU_SEQUENCE_HEADER` を保存する処理が、OBU サイズを LEB128 ではなく 1 バイト (`buffer[3]`) で読む。サイズが 128 バイト以上、または残バッファより大きいとき、`memcpy` が越境しシーケンスヘッダが壊れる。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp` の `SendFrame()` が AV1 で `buffer[2] == 0x0a` (`OBU_SEQUENCE_HEADER`) のとき `int obu_size = buffer[3];` とし、`memcpy(..., buffer + 2, obu_size + 2)` する
- コメントで LEB128 だと認めつつ「128 バイト以上になることはなさそう」としている
- 残サイズ (`size`) と `obu_size + 2` の比較が無い。`buffer[3]` が残バイトを超えると越境する
- 保存したヘッダはキーフレーム前に挿入されるため、壊れたヘッダは途中参加デコーダを壊す
- `hwenc_jetson` は momo 独自コードである

## 設計方針

- OBU サイズを LEB128 としてデコードする。1 バイト前提をやめる
- 読み取り前に残サイズを検証し、不足・不正ならヘッダ保存をスキップしてエラーログを出す
- キーフレームへの挿入は、保存済みヘッダが空なら挿入しない
- momo のみ修正する

## 完了条件

- OBU サイズが 1 バイトを超えても越境コピーしない
- 128 バイト未満の通常ヘッダは従来通り保存・挿入される
- 不正サイズではクラッシュせず、空のヘッダをキーフレームに差し込まない

## 解決方法

未着手 (PR 作成後に追記する)
