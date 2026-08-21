# zlib の Uncompress が倍々再試行でメモリを枯渇させる

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-zlib-uncompress-memory
- Polished: 2026-08-21
- Milestone: 2026.1.0

## 目的

`ZlibHelper::Uncompress` が展開後サイズの未知のまま 16KB から `output.size() * 2` で倍々に再試行するため、zip 爆弾相当の圧縮データ (DataChannel 経由で受信) でメモリが枯渇する。また失敗時に `throw std::exception()` を投げるが、これは唯一の呼び出し元 `SoraClient::OnMessage` で catch されず `std::terminate` になる。これを修正する。

## 現状

- `src/zlib_helper.h` の `ZlibHelper::Uncompress`: 展開後サイズの上限がなく、`Z_BUF_ERROR` のたびに `output.resize(output.size() * 2)` で無制限に拡張する
- 失敗時 (`Z_DATA_ERROR` 等) に `throw std::exception()` を投げるが、呼び出し元 `src/sora/sora_client.cpp` の `SoraClient::OnMessage` には try/catch がなく `std::terminate` になる
- 受信経路は Sora モードの DataChannel のみ。`SoraClient::OnMessage` は `compressed_labels_` (offer の `data_channels` で `compress: true` のラベル) に一致するラベルのメッセージを展開する。悪意のある、または障害を起こした Sora サーバ、および Sora を経由して中継されるピアが攻撃面となる

## 設計方針

- 展開後の最大サイズを絶対値の定数で制限する (例: 数 MB)。正当な DataChannel メッセージ (signaling の re-offer SDP や stats) の展開後サイズは高々数十 KB 程度であり、数 MB で十分に余裕がある
- 上限超過・データ破損などすべての失敗パスで例外を投げず、エラーを返す。`Uncompress` の `throw std::exception()` は撤廃する (送信側の `Compress` は自己生成データが対象で、本 issue の変更対象外)
- エラーの返し方は API の変更を伴う (例: `std::optional<std::string>` を返し、失敗時は `std::nullopt`) ため、唯一の呼び出し元 `SoraClient::OnMessage` でエラーをハンドリングする (エラーログを出力して該当メッセージを無視する等)
- 入力サイズから上限を計算する方式は、展開後サイズが入力サイズに対して理論上限を持たない zip 爆弾を防げないため採用しない
- 0015 の JSON 例外ハンドリング (SoraClient::OnMessage への try/catch 追加) とは独立に、Uncompress は例外を投げなくなるため、catch による握り潰しや実装順序への依存は生じない

## 完了条件

- zip 爆弾相当のデータでメモリ枯渇しない
- 異常データ (上限超過・データ破損) で `std::terminate` しない
- 正常な圧縮データの展開は従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
