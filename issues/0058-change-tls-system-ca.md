# TLS の証明書検証を OS のシステム CA に切り替える

- Created: 2026-09-10
- Completed: {YYYY-MM-DD}
- Branch: feature/change-tls-system-ca
- Polished: 2026-09-10

## 目的

momo の TLS 証明書検証は、ハードコードした ISRG Root X1、WebRTC 組込みルート (`rtc_base/ssl_roots.h`)、BoringSSL の既定パス (`X509_STORE_set_default_paths`) を組み合わせて信頼ストアを構成している。この方式は Windows では BoringSSL の既定パス (`/etc/ssl`) が存在しないため OS のルート証明書を読めず、ハードコードした証明書の有効期限や階層変更に追従し続ける必要がある。

sora-cpp-sdk は 2026.2.0 で既定の信頼ストアを OS のシステム CA に切り替え、ハードコードした証明書と `rtc_base/ssl_roots.h` 依存を撤廃した。`ca_cert` (指定した PEM のみを trust anchor にする仕組み) は以前からある。momo も同じ方式に揃え、OS が持つルート証明書を信頼の根拠にするとともに、`ca_cert` を CLI から指定できるようにする。

参考:

- sora-cpp-sdk 2026.2.0 の `[CHANGE] TLS 検証の信頼ストアを OS のシステム CA に切り替える`
- sora-cpp-sdk `issues/closed/0035-change-tls-trust-store-system-ca.md` (OS 別実装と CMake の差分)

## 現状

- `src/ssl_verifier.cpp` の `SSLVerifier::VerifyX509` は次の 3 つで信頼ストアを構成する
  - `SSLVerifier::AddCert` で `isrg_root` (ISRG Root X1) を追加する
  - `SSLVerifier::LoadBuiltinSSLRootCertificates` で WebRTC 組込みルート (`kSSLCertCertificateList`) を追加する
  - `X509_STORE_set_default_paths` で BoringSSL の既定パス (`/etc/ssl/cert.pem` / `/etc/ssl/certs`) を追加する
- `SSLVerifier::VerifyX509` はホスト名引数付きのオーバーロードを持ち、`VerifyHostname` で `X509_check_host` / `X509_check_ip` を行う
- `src/rtc/rtc_ssl_verifier.cpp` の `RTCSSLVerifier::VerifyChain` は `SSLVerifier::VerifyX509(x509, chain)` を呼ぶ。これが TURN-TLS / DTLS の検証経路になる
- `src/websocket.cpp` の `Websocket::InitWss` の verify コールバックは `SSLVerifier::VerifyX509(cert, chain, host)` を呼ぶ。`host` は WSS のホスト名検証に使う
- `src/rtc/rtc_manager.cpp` の `RTCManager::CreateConnection` は `RTCSSLVerifier(config_.insecure)` を `dependencies.tls_cert_verifier` に設定する
- `src/momo_args.h` の `MomoArgs` に `ca_cert` が無く、`--ca-cert` に相当するオプションも無い
- sora-cpp-sdk 2026.2.1 への同期 (`26d5bef7`) で `src/sora-cpp-sdk/src/ssl_verifier.cpp` と `src/sora-cpp-sdk/src/ssl_verifier/ssl_verifier_{android,ios,macos,ubuntu,windows}.*` が vendoring されている
  - `CMakeLists.txt` のビルド対象には含まれておらず、momo の検証には使われていない
- sora-cpp-sdk の `sora::SSLVerifier::VerifyX509` は `ca_cert` (PEM 文字列) を受け取り、未指定なら `sora::LoadSystemSSLRootCertificates`、指定時は指定された PEM のみを trust anchor にする
  - プラットフォームごとに `ssl_verifier_macos.cpp` (Security.framework)、`ssl_verifier_ubuntu.cpp` (`/etc/ssl/certs/ca-certificates.crt`)、`ssl_verifier_windows.cpp` (Windows ROOT ストア) を使う
  - WSS のホスト名検証は行わない

## 設計方針

1. sora-cpp-sdk の `SSLVerifier` 実装を momo でもビルドする。`src/sora-cpp-sdk/src/ssl_verifier.cpp` と、プラットフォームに対応する `src/sora-cpp-sdk/src/ssl_verifier/ssl_verifier_<os>.cpp` を `CMakeLists.txt` の `TARGET_OS` 分岐の `target_sources` に追加する
   - macOS は `ssl_verifier_macos.cpp`、Linux (Ubuntu / Raspberry Pi OS / Jetson) は `ssl_verifier_ubuntu.cpp`、Windows は `ssl_verifier_windows.cpp` を使う
   - iOS / Android は momo ではビルドしないため対象外とする
   - `src/sora-cpp-sdk/src/ssl_verifier/ssl_verifier_util.h` は各実装が include するヘッダであり、個別にビルド対象へ追加する必要はない
   - `ssl_verifier_macos.cpp` は Security.framework と CoreFoundation を使うため、`CMakeLists.txt` の macOS 分岐の `target_link_libraries` に `-framework Security` を追加し、コメントアウトされている `-framework CoreFoundation` を有効にする
   - `ssl_verifier_windows.cpp` は crypt32 を使うため、`CMakeLists.txt` の Windows 分岐の `target_link_libraries` でコメントアウトされている `crypt32.lib` を有効にする
2. momo の `SSLVerifier::VerifyX509` は信頼ストア構築を `sora::SSLVerifier::VerifyX509(x509, chain, ca_cert)` に委譲し、成功後に既存の `VerifyHostname` で WSS のホスト名検証を行う
   - `src/ssl_verifier.h` の `SSLVerifier` は `VerifyX509(X509* x509, STACK_OF(X509)* chain, const std::string& host, const std::optional<std::string>& ca_cert)` の 1 本に統一し、既存の 2 引数版と 3 引数版は削除する
   - `host` が空の場合はチェーン検証だけを行い、ホスト名検証は行わない。`RTCSSLVerifier::VerifyChain` は `host` に空文字を渡す
   - `VerifyHostname` は 0014 で追加した挙動であり、sora-cpp-sdk 側には無いため momo 側で維持する
3. `SSLVerifier::AddCert` と `SSLVerifier::LoadBuiltinSSLRootCertificates` を削除し、`rtc_base/ssl_roots.h` と `isrg_root` への依存をなくす
4. `ca_cert` を追加し、次の経路に通す
   - `MomoArgs::ca_cert` は `std::string` とし、`--ca-cert` (PEM ファイルのパス、空文字で未指定) を受け取る。`main.cpp` でファイルを読み込み、PEM 文字列を `std::optional<std::string>` として各 config に設定する
   - `RTCManagerConfig::ca_cert` (`std::optional<std::string>`) に設定し、`RTCManager::CreateConnection` で `RTCSSLVerifier(config_.insecure, config_.ca_cert)` に渡す (TURN-TLS / DTLS)
   - `SoraClientConfig::ca_cert` と `AyameClientConfig::ca_cert` (`std::optional<std::string>`) に設定し、`Websocket` の ssl_tag / https_proxy_tag コンストラクタ、`InitWss`、`SSLVerifier::VerifyX509` に渡す (WSS)
   - 変更対象は `src/ssl_verifier.h` / `src/ssl_verifier.cpp` / `src/rtc/rtc_ssl_verifier.h` / `src/rtc/rtc_ssl_verifier.cpp` / `src/websocket.h` / `src/websocket.cpp` / `src/rtc/rtc_manager.h` / `src/rtc/rtc_manager.cpp` / `src/sora/sora_client.h` / `src/sora/sora_client.cpp` / `src/ayame/ayame_client.h` / `src/ayame/ayame_client.cpp` / `src/momo_args.h` / `src/util.cpp` / `src/main.cpp` / `CMakeLists.txt`
5. `--ca-cert` は `--client-cert` / `--client-key` と同じくファイルパスを取る。指定時は指定した PEM のみを trust anchor とし、未指定時は OS のシステム CA を使う (sora-cpp-sdk と同じ挙動)
6. libwebrtc の組込みルート (`rtc_base/ssl_roots.h`) を利用する設定は設けない。sora-cpp-sdk が組込みルート依存を撤廃しており、信頼の根拠を OS のシステム CA と `--ca-cert` の 2 系統に統一するため
7. `--insecure` の挙動は変えない。指定時はチェーン検証とホスト名検証をスキップする

## 完了条件

- TURN-TLS / DTLS の検証が `sora::SSLVerifier` のシステム CA 方式で行われる
- WSS の検証でホスト名検証が維持される
- `--ca-cert` に PEM ファイルを指定すると、その PEM のみを trust anchor として検証する
- `--ca-cert` 未指定時は OS のシステム CA を信頼する
- `--insecure` 指定時は検証をスキップする
- `src/ssl_verifier.cpp` から `isrg_root` と `rtc_base/ssl_roots.h` 依存が消えている
- `CMakeLists.txt` の macOS / Windows 分岐に `-framework Security` / `-framework CoreFoundation` / `crypt32.lib` が追加されている
- macOS / Windows / Linux (Ubuntu / Raspberry Pi OS / Jetson) の全対象プラットフォームでビルドが通る
- Sora / Ayame / P2P の E2E テストが通る

## 解決方法

{YYYY-MM-DD} に追記する
