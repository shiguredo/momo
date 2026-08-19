# 埋め込まれた Let's Encrypt R3 中間証明書が失効済みのため削除する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-expired-r3-certificate
- Polished: {YYYY-MM-DD}

## 目的

`src/ssl_verifier.cpp` にハードコードされた Let's Encrypt R3 中間証明書が `NotAfter: 2025-09-15` で失効済みである。trust anchor として直接 `X509_STORE_add_cert` に積んでいるため現状は接続が壊れていないが、中間証明書をルートとして pin する設計自体が誤解を招き、OpenSSL の挙動変更や別検証経路で接続障害になる潜在リスクがある。失効済みの証明書を削除する。

## 現状

- `src/ssl_verifier.cpp` の `lets_encrypt_r3` (57-90 行) の notAfter が `250915160000Z` = 2025-09-15 で失効済み
- `VerifyX509()` が `AddCert(lets_encrypt_r3, store)` (191 行) で trust anchor として追加
- `isrg_root` (2035 年まで) は有効

## 設計方針

- `lets_encrypt_r3` の定義と `AddCert` 呼び出しを削除する
- 中間証明書はサーバ側のチェーンで提供されるものであり、クライアント側に pin する必要はない
- `isrg_root` と WebRTC 組込みルート・システム既定パスで検証を続ける

## 完了条件

- `src/ssl_verifier.cpp` から失効済み R3 証明書が削除されている
- WSS 接続の E2E テストが通る

## 解決方法

未着手 (PR 作成後に追記する)
