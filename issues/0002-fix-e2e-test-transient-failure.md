# 一過性の理由により E2E テストがタイムアウトする

- Priority: Low
- Created: 2026-06-16
- Model: deepseek-v4-flash
- Completed: {YYYY-MM-DD}
- Branch: なし（コード修正を伴わない記録のため）
- Polished: 2026-08-19

## 目的

コード回帰と一過性の要因を区別できなければ、無駄な調査工数が発生する。そのため、今回観測された障害パターンを記録し、再発時の初動判断に役立てる。

## 優先度根拠

- Low とする。
- 後続 run #534 が全テスト PASS しており（momo 本体のコード差分なし）、コード回帰の可能性が低いため。

## 現状

### 発生した現象

Apple Video Toolbox (macOS) の E2E テスト 6 件全てが同時にタイムアウトした (run #533, 2026-06-15 03:17 UTC)。このジョブの failure により matrix strategy の他ジョブも全てキャンセルされた。

### 障害の詳細

- **エラー種別**: `httpx.ReadTimeout: timed out`
- **エラー発生箇所**: `Momo.__enter__` → `_wait_for_startup()` 内の `client.get(url, timeout=5)` による metrics エンドポイントへの HTTP GET。`httpx.ReadTimeout` は `_wait_for_startup` 内の `except` 節 (`ConnectError`, `ConnectTimeout`, `HTTPStatusError` のみ捕捉) で捕捉されず、`__enter__` の `except Exception` に伝播する。
- **タイムアウト値**: 個々の HTTP GET は 5 秒でタイムアウト。`_wait_for_startup` 全体のループは最大 30 秒だが、捕捉漏れによりループが継続できず早期終了する。
- **該当テスト**: `test_connection_stats[H264]`, `test_connection_stats[H265]`, `test_simulcast[H264-SimulcastEncoderAdapter (VideoToolbox, VideoToolbox, VideoToolbox)]`, `test_simulcast[H265-SimulcastEncoderAdapter (VideoToolbox, VideoToolbox, VideoToolbox)]`, `test_sora_sendonly_recvonly_pair[H264]`, `test_sora_sendonly_recvonly_pair[H265]`

### 確認できたこと

- 後続 run #534 (feature/update-libwebrtc-m150 をマージ済みの feature/fix-applevt-e2e-logging) では 3 時間 20 分後に全テスト PASS (Apple Video Toolbox ジョブ所要 2 分)
- #533 と #534 の間のコード差分は build.yml のパスフィルタ削除と test/momo.py のプロセス出力の CI ログ表示のみで、momo 本体 (`src/`) のビルド成果物に影響なし
- develop ブランチの直近 E2E テスト (#529, #530) は全て成功

### 推定原因

起票時に確認した CI ログでは最初の 1 回目のポーリングで metrics エンドポイントが 200 OK を返した (stats は空) ことが確認されている。その後、後続のポーリングで `httpx.ReadTimeout` が発生しており、metrics HTTP サーバが起動直後に応答を停止したことを示す。なお、このログは GitHub Actions のログ保持期限切れにより再取得できないため、この観察は起票時の記録に基づく。

momo は boost::asio の単一スレッド非同期イベントループで動作しており、シグナリング WebSocket 接続が非同期に実装されているため、シグナリングサーバ未応答だけが原因で metrics サーバ全体が停止するとは考えにくい。momo プロセス内で何らかの異常 (WebRTC 内部のデッドロック、リソース枯渇、クラッシュ等) が発生した可能性もあるが、本障害時の stderr ログは取得できておらず、原因の特定には至っていない。

self-hosted ランナーの負荷状況やシグナリングサーバ側のログも取得できていないため、複数の要因が関与した可能性が残る。

## 設計方針

コード修正は行わず、記録として残す。現時点で対応する修正箇所はない。障害の詳細で触れた `httpx.ReadTimeout` の捕捉漏れは既知のテストハーネスの欠陥であり、今回の障害の主因ではないため修正対象としない。

## 完了条件

本 issue を起票者が最終確認し、事実関係に誤りがないことを確認した上で `issues/closed/` へ移動する。

## 解決方法

コード修正は不要。後続 run が momo 本体のコード差分なしで全テスト PASS しており、コード回帰の可能性が低いため一過性と判断する。`pytest-rerunfailures` (reruns=3, reruns_delay=1) が導入済みで、秒〜分オーダーの短時間の瞬断はリトライで吸収されるが、今回の事象はリトライを全て消費して失敗しておりリトライでは吸収できないクラスである。捕捉漏れ (`httpx.ReadTimeout` が `_wait_for_startup` の `except` 節で捕捉されず早期終了する) が障害を増幅した可能性はあるが、metrics サーバ自体の応答停止が主因であり、捕捉漏れを修正しても 30 秒ループ内で復帰しなければ失敗する点は変わらない。

### 再発時の対応手順

1. `e2e-test.yml` の `E2E Test Apple Video Toolbox` ジョブのログで `httpx.ReadTimeout` と `Metrics endpoint is up but stats is empty` が同時に出現していることを確認
2. ジョブログの momo プロセス出力で stderr を確認し、momo プロセス自体の異常 (クラッシュ等) が無いかを確認する (2026-06-16 以降、momo の stdout/stderr は CI ログに表示される。ただし open issue 0036 が stderr の収集方式を PIPE に変更する予定のため、実装後はこの前提が変わり得る点に注意する)。異常 (クラッシュ等) を確認した場合は一過性ではなく実バグの可能性が高いため、即座に別 issue を起票する
3. 同一コード (momo 本体のコード差分がない run) で別 run が PASS しているか GitHub Actions 上で確認。確認できない場合は step 6 へ進む
4. 上記 1〜3 がすべて確認できれば一過性障害の可能性が高いと判断し、該当 run を再実行する
5. 再実行した run が再び失敗した場合は状況が変化したと判断し、状況を記録して別 issue を起票する（同一パターン (`httpx.ReadTimeout` + stats 空) か否かを問わない）
6. 同一コードで PASS している別 run が確認できない状況で、`pytest-rerunfailures` (reruns=3) のリトライ回数を超過しても同一パターンが継続する場合、状況が変化したと判断し別 issue を起票する
