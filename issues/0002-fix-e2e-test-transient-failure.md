# 一過性の理由により E2E テストがタイムアウトする

- Priority: Low
- Created: 2026-06-16
- Model: deepseek-v4-flash
- Polished: 2026-06-16

## 目的

コード回帰と一過性の要因を区別できなければ、無駄な調査工数が発生する。そのため、今回観測された障害パターンを記録し、再発時の初動判断に役立てる。

## 優先度根拠

- Low とする。
- 同一コードで 3.5 時間後に全テストが PASS しており、コード回帰の可能性は低い。
- 既に `pytest-rerunfailures` (reruns=3, reruns_delay=1) が導入済みであり、同程度の瞬断であればリトライで吸収可能。

## 現状

### 発生した現象

Apple Video Toolbox (macOS) の E2E テスト 6 件全てが同時にタイムアウトした (run #533, 2026-06-15 03:17 UTC)。このジョブの failure により matrix strategy の他ジョブも全てキャンセルされた。

### 障害の詳細

- **エラー種別**: `httpx.ReadTimeout: timed out`
- **エラー発生箇所**: `Momo.__enter__` → `_wait_for_startup()` (test/momo.py:697) の `client.get(url, timeout=5)` による metrics エンドポイントへの HTTP GET。`httpx.ReadTimeout` は `_wait_for_startup` 内の `except` 節 (`ConnectError`, `ConnectTimeout`, `HTTPStatusError` のみ捕捉) で捕捉されず、`__enter__` の `except Exception` に伝播する。
- **タイムアウト値**: 個々の HTTP GET は 5 秒でタイムアウト。`_wait_for_startup` 全体のループは最大 30 秒だが、捕捉漏れによりループが継続できず早期終了する。
- **該当テスト**: `test_connection_stats[H264]`, `test_connection_stats[H265]`, `test_simulcast[H264/SimulcastEncoderAdapter]`, `test_simulcast[H265/SimulcastEncoderAdapter]`, `test_sora_sendonly_recvonly_pair[H264]`, `test_sora_sendonly_recvonly_pair[H265]`

### 確認できたこと

- 同じブランチ構成 (feature/update-libwebrtc-m150) の後続 run #534 では 3.5 時間後に全テスト PASS (所要 2 分)
- コード差分は run.py のコメント除去のみで、ビルド成果物に影響なし
- develop ブランチの直近 E2E テスト (#529, #530) は全て成功

### 推定原因

CI ログでは最初の 1 回目のポーリングで metrics エンドポイントが 200 OK を返した (stats は空) ことが確認されている。その後、後続のポーリングで `httpx.ReadTimeout` が発生しており、metrics HTTP サーバが起動直後に応答を停止したことを示す。

momo は boost::asio の単一スレッド非同期イベントループで動作しており、シグナリング WebSocket 接続が非同期に実装されているため、シグナリングサーバ未応答だけが原因で metrics サーバ全体が停止するとは考えにくい。momo プロセス内で何らかの異常 (WebRTC 内部のデッドロック、リソース枯渇、クラッシュ等) が発生した可能性もあるが、本障害時の stderr ログは取得できておらず、原因の特定には至っていない。

self-hosted ランナーの負荷状況やシグナリングサーバ側のログも取得できていないため、複数の要因が関与した可能性が残る。

## 設計方針

コード修正は行わず、記録として残す。原因特定には至っていないが、同一コードで後続 run が PASS していることからコード回帰の可能性は低く、現時点で対応する修正箇所はない。

## 完了条件

本 issue を起票者が最終確認し、事実関係に誤りがないことを確認した上で `issues/closed/` へ移動する。

## 解決方法

コード修正は不要。`pytest-rerunfailures` (reruns=3, reruns_delay=1) が既に導入されており、同程度の瞬断はリトライで吸収可能と判断する。

### 再発時の対応手順

1. `e2e-test.yml` の `E2E Test Apple Video Toolbox` ジョブのログで `httpx.ReadTimeout` と `Metrics endpoint is up but stats is empty` が同時に出現していることを確認
2. 同一コードで別 run が PASS しているか確認 (同じブランチの他 run を GitHub Actions 上で確認)
3. 上記 2 点が一致すれば一過性障害の可能性が高いと判断し、該当 run を再実行する
4. `pytest-rerunfailures` (reruns=3) のリトライ回数を超過しても同一パターンが継続する場合、状況が変化したと判断し別 issue を起票する
