#ifndef SORA_SSL_VERIFIER_UTIL_H_
#define SORA_SSL_VERIFIER_UTIL_H_

#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

// OpenSSL
#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/pem.h>
#include <openssl/stack.h>
#include <openssl/x509.h>

// WebRTC
#include <rtc_base/logging.h>

namespace sora {

// スコープ終了時に任意の処理を実行する RAII ガード
struct Guard {
  std::function<void()> f;
  Guard(std::function<void()> f) : f(std::move(f)) {}
  ~Guard() { f(); }
  Guard(const Guard&) = delete;
  Guard& operator=(const Guard&) = delete;
};

// X509_STORE_add_cert の戻り値を解釈する。
// kSuccess: 追加成功
// kDuplicate: X509_R_CERT_ALREADY_IN_HASH_TABLE 重複（エラーキューはクリア済み）
// kError: その他のエラー（エラーキューは未クリア、呼び出し元で取得してからクリアすること）
enum class AddCertResult {
  kSuccess,
  kDuplicate,
  kError,
};

inline AddCertResult CheckAddCertResult(X509* cert, X509_STORE* store) {
  int r = X509_STORE_add_cert(store, cert);
  if (r != 0) {
    return AddCertResult::kSuccess;
  }
  unsigned long err = ERR_peek_last_error();
  if (ERR_GET_REASON(err) == X509_R_CERT_ALREADY_IN_HASH_TABLE) {
    ERR_get_error();
    return AddCertResult::kDuplicate;
  }
  return AddCertResult::kError;
}

// X509_STORE_add_cert を実行し、エラー処理を共通化する。
// 成功時は true を返す。
// X509_R_CERT_ALREADY_IN_HASH_TABLE の重複は無視し false を返す（WARNING ログなし）。
// それ以外のエラーは subject を含む LS_WARNING ログを出力し false を返す。
// 呼び出し元は cert の所有権を保持し、別途 X509_free すること。
inline bool TryAddCertToStore(X509* cert,
                              X509_STORE* store,
                              const char* context) {
  AddCertResult result = CheckAddCertResult(cert, store);
  if (result == AddCertResult::kSuccess) {
    return true;
  }
  if (result == AddCertResult::kDuplicate) {
    return false;
  }
  char subject[256] = {0};
  X509_NAME_oneline(X509_get_subject_name(cert), subject, sizeof(subject));
  RTC_LOG(LS_WARNING) << context
                      << ": X509_STORE_add_cert failed: subject=" << subject;
  ERR_get_error();
  return false;
}

// PEM 形式の文字列から X509 証明書のリストをパースする
// 戻り値の各 X509* は呼び出し元で X509_free すること
inline std::vector<X509*> ParsePEMCerts(const std::string& pem) {
  std::vector<X509*> certs;
  BIO* bio = BIO_new_mem_buf(pem.c_str(), pem.size());
  if (bio == nullptr) {
    ERR_get_error();
    return certs;
  }
  Guard bio_guard([bio]() { BIO_free(bio); });
  while (true) {
    X509* cert = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr);
    if (cert == nullptr) {
      ERR_get_error();
      break;
    }
    certs.push_back(cert);
  }
  return certs;
}

// 証明書とチェーンの subject / issuer を INFO ログに出力する
inline void DumpX509CertificateInfo(X509* x509, STACK_OF(X509) * chain) {
  char data[256] = {0};
  RTC_LOG(LS_INFO) << "cert:";
  X509_NAME_oneline(X509_get_subject_name(x509), data, sizeof(data));
  RTC_LOG(LS_INFO) << "  subject = " << data;
  X509_NAME_oneline(X509_get_issuer_name(x509), data, sizeof(data));
  RTC_LOG(LS_INFO) << "  issuer  = " << data;

  if (chain != nullptr) {
    size_t n = sk_X509_num(chain);
    for (size_t i = 0; i < n; i++) {
      X509* x = sk_X509_value(chain, i);
      RTC_LOG(LS_INFO) << "chain[" << i << "]:";
      X509_NAME_oneline(X509_get_subject_name(x), data, sizeof(data));
      RTC_LOG(LS_INFO) << "  subject = " << data;
      X509_NAME_oneline(X509_get_issuer_name(x), data, sizeof(data));
      RTC_LOG(LS_INFO) << "  issuer  = " << data;
    }
  }
}

}  // namespace sora

#endif
