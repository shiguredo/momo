# TLS 証明書検証の動作確認を pytest E2E テストにする

- Created: 2026-09-16
- Completed: {YYYY-MM-DD}
- Branch: feature/add-tls-verification-e2e-test
- Polished: {YYYY-MM-DD}

## 目的

TLS のシステム CA 切り替えと `--ca-cert` は実装済みだが、信頼ストアの分岐（未指定 / `--ca-cert` / `--insecure` / TURN-TLS）は CI 成果物での手動確認に留まっている。回帰を自動で検出するため、同じ確認を既存の pytest E2E に載せる。

## 現状

- `src/ssl_verifier.cpp` の `SSLVerifier::VerifyX509` はチェーン検証を `sora::SSLVerifier::VerifyX509` に委譲し、成功後に `VerifyHostname` で WSS のホスト名検証を行う
- `--ca-cert` は `src/util.cpp` で受け取り、`src/main.cpp` の `LoadCaCertPem` が PEM を読み、`RTCManagerConfig` / `SoraClientConfig` / `AyameClientConfig` 経由で WSS と TURN-TLS / DTLS に渡す
- 既存の Sora / Ayame E2E (`test/test_sora_mode.py` / `test/test_ayame_mode.py` 等) は通常のシステム CA 経路で接続できることしか見ておらず、`--ca-cert` / 自己発行 CA / TURN-TLS の `relayProtocol` は検証していない
- `test/momo.py` の `Momo` は `--client-cert` / `--client-key` / `--insecure` を渡せるが、`--ca-cert` を渡す引数が無い
- 手動確認では次を確認済みである
  - 未指定 / ISRG Root X1 の PEM / 自己発行 + `--insecure` で WSS (Ayame Labo) と TURN-TLS (Sora Labo の `turn_tls_only`) に接続できる
  - TURN-TLS では metrics の `relayProtocol` が `tls` になる
  - 自己発行のみでは `CERTIFICATE_VERIFY_FAILED` になる
  - Ubuntu 24.04 でシステム CA から ISRG Root X1 / X2 を外すと未指定が失敗し、`--ca-cert` と `--insecure` は通る

## 設計方針

- モックやスタブは使わない。実プロセスの `momo` と実シグナリング (既存 E2E と同じ環境変数) で確認する
- `test/momo.py` の `Momo` に `--ca-cert` を渡せる引数を追加する
- 成功系と失敗系の両方を pytest で書く
  - 成功: 未指定、`--ca-cert` に公開の ISRG Root X1 PEM、自己発行 CA + `--insecure`
  - 失敗: 自己発行 CA のみ
  - TURN-TLS: Sora モードで `turn_tls_only` 相当の設定を使い、接続後に metrics の `relayProtocol` が `tls` であることを確認する
- PEM や一時 CA はテスト内で用意する。実値の秘密情報は issue / テストに書かない
- システム CA バンドルから特定ルートを外す確認は CI ランナーの権限・再現性が低いため、本 issue の必須対象外とする。必要なら別 issue にする
- WSS のホスト名不一致の確認は本 issue の対象外とする
- 既存の Sora / Ayame 接続 E2E の置き換えはしない。TLS 検証専用のテストファイルを追加する

## 完了条件

- `test/momo.py` から `--ca-cert` を指定できる
- 未指定 / `--ca-cert` / 自己発行 + `--insecure` の成功が pytest で確認できる
- 自己発行のみの失敗が pytest で確認できる
- TURN-TLS 経路で metrics の `relayProtocol` が `tls` であることが pytest で確認できる
- 追加した E2E が CI (`e2e-test.yml` の既存経路) で通る
