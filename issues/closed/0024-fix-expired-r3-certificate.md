# 埋め込まれた Let's Encrypt R3 中間証明書が期限切れのため削除する

- Created: 2026-08-19
- Completed: 2026-08-21
- Branch: feature/fix-expired-r3-certificate
- Polished: 2026-08-20
- Milestone: 2026.1.0

## 目的

`src/ssl_verifier.cpp` にハードコードされた Let's Encrypt R3 中間証明書は `NotAfter: 2025-09-15` で期限切れである。`X509_STORE_add_cert` で trust anchor として積んでいるが、実際の検証は `isrg_root` とサーバが提供するチェーンで成立しており、現在の検証経路で R3 は使用されない。また R3 自体が期限切れのため、R3 を含むチェーンは 2025-09-15 以降検証不能であり、R3 の pin は検証に寄与しない。期限切れの証明書を trust anchor として残す設計は誤解を招き、BoringSSL の挙動変更や別検証経路で接続障害になる潜在リスクがある。期限切れの証明書を削除する。

## 現状

- `src/ssl_verifier.cpp` の `lets_encrypt_r3` の notAfter が `250915160000Z` = 2025-09-15 で期限切れ
- `VerifyX509()` が `AddCert(lets_encrypt_r3, store)` で trust anchor として追加
- `isrg_root` (2035 年まで) は有効
- 同一ファイルを変更する 0014 と並行しており、0014 は本 issue を先にマージしてから rebase する順序を推奨している

## 設計方針

- `lets_encrypt_r3` の定義と `AddCert` 呼び出しを削除する
- 中間証明書はサーバ側のチェーンで提供されるものであり、クライアント側に pin する必要はない
- `isrg_root` と WebRTC 組込みルート・システム既定パスで検証を続ける

## 完了条件

- `src/ssl_verifier.cpp` から期限切れ R3 証明書が削除されている
- WSS 接続の E2E テストが通る

## 解決方法

- `src/ssl_verifier.cpp` から `lets_encrypt_r3` 定数を削除した
- `VerifyX509()` 内の `AddCert(lets_encrypt_r3, store)` 呼び出しを削除した
- `isrg_root` と WebRTC 組込みルート・システム既定パスによる検証はそのまま残した
- CI の全プラットフォームビルドおよび WSS を含む E2E が通過したことを確認した
- PR: https://github.com/shiguredo/momo/pull/462
