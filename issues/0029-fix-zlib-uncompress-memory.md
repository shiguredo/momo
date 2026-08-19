# zlib の Uncompress が倍々再試行でメモリを枯渇させる

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-zlib-uncompress-memory
- Polished: {YYYY-MM-DD}
- Milestone: 2026.1.0

## 目的

`ZlibHelper::Uncompress` が展開後サイズの未知のまま 16KB から `output.size() * 2` で倍々に再試行するため、zip 爆弾相当の圧縮データ (DataChannel 経由で受信) でメモリが枯渇する。また失敗時に `throw std::exception()` を投げるが、これは呼び出し側で catch されず `std::terminate` になる。これを修正する。

## 現状

- `src/zlib_helper.h` の `Uncompress()` (43-62 行): `Z_BUF_ERROR` のたびに `output.resize(output.size() * 2)` で無制限に拡張
- 呼び出し元は `src/sora/sora_client.cpp` (DataChannel 受信データの展開)
- 例外 (`throw std::exception()`) が未処理で `std::terminate`

## 設計方針

- 展開後の最大サイズを制限する (例: 数 MB) か、入力サイズから妥当な上限を計算する
- 上限超過時はエラーを返し、例外を投げない
- 呼び出し元でエラーをハンドリングする (該当データを無視する等)

## 完了条件

- zip 爆弾相当のデータでメモリ枯渇しない
- 異常データで `std::terminate` しない
- 正常な圧縮データの展開は従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
