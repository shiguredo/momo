// OpenSSL
#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/pem.h>
#include <openssl/x509.h>

// WebRTC
#include <rtc_base/logging.h>

#include "ssl_verifier_util.h"

namespace sora {

bool LoadSystemSSLRootCertificates(X509_STORE* store) {
  constexpr const char* kCACertPath = "/etc/ssl/certs/ca-certificates.crt";
  BIO* bio = BIO_new_file(kCACertPath, "r");
  if (bio == nullptr) {
    ERR_get_error();
    RTC_LOG(LS_ERROR)
        << "LoadSystemSSLRootCertificates: BIO_new_file failed: path="
        << kCACertPath;
    return false;
  }
  // BIO の解放は必ず通す
  // bio は以降再代入されない前提で値捕捉する
  Guard bio_guard([bio]() { BIO_free(bio); });

  int added = 0;
  while (true) {
    X509* cert = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr);
    if (cert == nullptr) {
      ERR_get_error();
      break;
    }
    if (TryAddCertToStore(cert, store, kCACertPath)) {
      ++added;
    }
    X509_free(cert);
  }

  if (added == 0) {
    RTC_LOG(LS_ERROR)
        << "LoadSystemSSLRootCertificates: no certificates loaded: path="
        << kCACertPath;
    return false;
  }
  RTC_LOG(LS_INFO) << "LoadSystemSSLRootCertificates: added=" << added;
  return true;
}

}  // namespace sora
