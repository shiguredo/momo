#include "sora/ssl_verifier.h"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

// OpenSSL
#include <openssl/err.h>
#include <openssl/stack.h>
#include <openssl/x509.h>

// WebRTC
#include <rtc_base/logging.h>

#include "ssl_verifier/ssl_verifier_util.h"

namespace sora {

// 各プラットフォームの ssl_verifier_<os>.cpp で定義される
bool LoadSystemSSLRootCertificates(X509_STORE* store);

namespace {

// PEM 形式の文字列からルート証明書を読み込んでストアに追加する
// 1 件も追加できなかった場合は false を返す
bool LoadCertsFromPEM(const std::string& pem, X509_STORE* store) {
  std::vector<X509*> certs = ParsePEMCerts(pem);
  bool added = false;
  for (X509* cert : certs) {
    if (TryAddCertToStore(cert, store, "LoadCertsFromPEM")) {
      added = true;
    }
    X509_free(cert);
  }
  return added;
}

}  // namespace

bool SSLVerifier::VerifyX509(X509* x509,
                             STACK_OF(X509) * chain,
                             const std::optional<std::string>& ca_cert) {
  DumpX509CertificateInfo(x509, chain);

  X509_STORE* store = nullptr;
  X509_STORE_CTX* ctx = nullptr;

  Guard guard([&]() {
    // nullptr を渡しても何もしない
    X509_STORE_CTX_free(ctx);
    X509_STORE_free(store);
  });

  store = X509_STORE_new();
  if (store == nullptr) {
    RTC_LOG(LS_ERROR) << "X509_STORE_new failed";
    return false;
  }
  int r;
  r = X509_STORE_set_flags(store, X509_V_FLAG_TRUSTED_FIRST);
  if (r == 0) {
    RTC_LOG(LS_ERROR) << "X509_STORE_set_flags failed";
    return false;
  }

  if (!ca_cert) {
    // OS のシステム CA を信頼ストアに追加する
    if (!LoadSystemSSLRootCertificates(store)) {
      RTC_LOG(LS_ERROR) << "LoadSystemSSLRootCertificates failed";
      return false;
    }
  } else {
    // ルート証明書が指定されている場合、その証明書以外は読み込まない
    if (!LoadCertsFromPEM(*ca_cert, store)) {
      RTC_LOG(LS_ERROR) << "Failed to add ca_cert: ca_cert_length="
                        << ca_cert->size();
      return false;
    }
  }
  ctx = X509_STORE_CTX_new();
  if (ctx == nullptr) {
    RTC_LOG(LS_ERROR) << "X509_STORE_CTX_new failed";
    return false;
  }
  r = X509_STORE_CTX_init(ctx, store, x509, chain);
  if (r == 0) {
    RTC_LOG(LS_ERROR) << "X509_STORE_CTX_init failed";
    return false;
  }
  r = X509_verify_cert(ctx);
  if (r <= 0) {
    RTC_LOG(LS_INFO) << "X509_verify_cert failed: r=" << r << " message="
                     << X509_verify_cert_error_string(
                            X509_STORE_CTX_get_error(ctx));
    return false;
  }
  return true;
}

}  // namespace sora
