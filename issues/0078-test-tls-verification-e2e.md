# TLS 証明書検証の動作確認を pytest E2E テストにする

- Created: 2026-09-16
- Completed: {YYYY-MM-DD}
- Branch: feature/add-tls-verification-e2e-test
- Polished: 2026-09-17

## 目的

TLS のシステム CA 切り替えと `--ca-cert` は実装済みだが、信頼ストアの分岐（未指定 / `--ca-cert` / `--insecure`）と TURN-TLS 経路は CI 成果物での手動確認に留まっている。回帰を自動で検出するため、同じ確認を既存の pytest E2E に載せる。

## 現状

- `src/ssl_verifier.cpp` の `SSLVerifier::VerifyX509` はチェーン検証を `sora::SSLVerifier::VerifyX509` に委譲し、成功後に `VerifyHostname` で WSS のホスト名検証を行う
- `--ca-cert` は `src/util.cpp` で受け取り、`src/main.cpp` の `LoadCaCertPem` が PEM を読み、`RTCManagerConfig` / `SoraClientConfig` / `AyameClientConfig` 経由で WSS と TURN-TLS / DTLS に渡す
- WSS は `Websocket` → `SSLVerifier::VerifyX509(..., host, ca_cert)`（ホスト名検証あり）。TURN-TLS / DTLS は `RTCSSLVerifier::VerifyChain` → `VerifyX509(..., "", ca_cert)`（ホスト名検証なし）。別コードパスである
- 既存の Sora / Ayame E2E (`test/test_sora_mode.py` / `test/test_ayame_mode.py` 等) は通常のシステム CA 経路で接続できることしか見ておらず、`--ca-cert` / 自己発行 CA / TURN-TLS の `relayProtocol` は検証していない
- `test/momo.py` の `Momo` は `--client-cert` / `--client-key` / `--insecure` を渡せるが、`--ca-cert` を渡す引数が無い
- `Momo` は `stdout=None` / `stderr=None`（親へ継承）のため、pytest からプロセスログ文字列を読めない。TLS 失敗時も Ayame / Sora とも `ReconnectAfter` で再接続し、プロセスは即終了しない
- `_wait_for_startup` は Sora では metrics の `stats` 非空を待つが、Ayame / P2P では metrics の HTTP 200 だけで成功とみなす。Ayame で「起動できた」だけでは WSS ハンドシェイク成功を証明しない
- 手動確認（0058）では次を確認済みである
  - 未指定 / ISRG Root X1 の PEM / 自己発行 + `--insecure` で WSS (Ayame Labo) と TURN-TLS (Sora Labo の `turn_tls_only`) に接続できる
  - TURN-TLS では metrics の `relayProtocol` が `tls` になる
  - 自己発行のみでは証明書検証に失敗する
  - システム CA から特定ルートを外す確認と WSS ホスト名不一致は手動では見たが、本 issue の必須対象外とする（権限・再現性、または別経路のため）

## 設計方針

- モックやスタブは使わない。実プロセスの `momo` と実シグナリング (既存 E2E と同じ `TEST_SORA_MODE_*` 環境変数) で確認する
- `test/momo.py` の `Momo` に `--ca-cert` を渡せる引数 (`ca_cert`) を追加する
- 既存の Sora / Ayame 接続 E2E の置き換えはしない。TLS 検証専用のテストファイル（例: `test/test_tls_verification.py`）を追加する
- モード分担
  - 信頼ストア分岐（成功・失敗）: **Sora モードの通常接続**（TURN 強制なし）。Sora は `_wait_for_startup` が `stats` 非空を要求し、接続未確立を失敗として扱いやすい
  - TURN-TLS: **Sora モードのみ**。信頼ストアは未指定（システム CA）に限定し、`relayProtocol == "tls"` を確認する。信頼ストアの組み合わせ網羅は通常接続側で行う
- 成功・失敗の観測（ログ文字列やプロセス即終了は使わない）
  - 成功: `with Momo(...)` で入場できたうえで `wait_for_connection` が成功すること（必要なら `get_metrics(wait_stats=...)` で補強）
  - 失敗（自己発行 CA のみ）: `with Momo(...)` の入場自体が失敗すること。Sora では `__enter__` 内の `_wait_for_startup` が `stats` 非空を待つため、TLS 失敗で接続が立たないとタイムアウトして `RuntimeError`（`momo process failed to start within ...`）になる。`wait_for_connection` の失敗では見ない（入場前に例外になり到達しない）。コーデック不正のような「プロセス即終了」パターンの写経はしない
- 成功ケース
  - 未指定（システム CA）
  - `--ca-cert` に Let's Encrypt が公開している ISRG Root X1 の PEM（公開物。テスト内で取得するかフィクスチャとして置く）
  - 自己発行 CA + `--insecure`
- 失敗ケース
  - 自己発行 CA のみ（`--insecure` なし）
- TURN-TLS
  - Sora Labo の JWT プライベートクレーム `turn_tls_only: true` を `access_token` に載せる（`metadata` 直下への `turn_tls_only` 指定は Labo では廃止済み）
  - `test/conftest.py` の `sora_settings` は流用しつつ、当該テストだけ JWT payload に `turn_tls_only: true` を足す（共通 fixture を壊さない）
  - 接続後、metrics の `stats` から `type == "local-candidate"` かつ `candidateType == "relay"` のエントリを探し、`relayProtocol == "tls"` であることを確認する
- PEM / 自己発行 CA はテスト内で用意する。自己発行は `openssl` で一時ファイルを生成する（追加の Python 依存は増やさない）。秘密鍵や接続用シークレットの実値は issue / テストに書かない
- システム CA バンドルから特定ルートを外す確認、および WSS のホスト名不一致は本 issue の必須対象外とする。必要なら別 issue にする

## 完了条件

- `test/momo.py` から `--ca-cert` を指定できる
- Sora 通常接続で、未指定 / `--ca-cert` (ISRG Root X1) / 自己発行 + `--insecure` の成功を `wait_for_connection` 等で確認できる
- Sora 通常接続で、自己発行のみのとき `with Momo(...)` が `_wait_for_startup` タイムアウト由来の `RuntimeError` になること（ログ文字列マッチやプロセス即終了アサーションに依存しない）
- Sora + JWT `turn_tls_only` で TURN-TLS に接続し、metrics の relay candidate の `relayProtocol` が `tls` であることを確認できる（信頼ストアは未指定）
- 追加した E2E が CI (`e2e-test.yml` の既存 `pytest .` 経路) で通る
