#include "ssl_verifier.h"

// webrtc
#include <rtc_base/logging.h>
#include <rtc_base/openssl_utility.h>
#include <rtc_base/ssl_roots.h>

// openssl
#include <openssl/x509v3.h>

// boost
#include <boost/asio/connect.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>

bool SSLVerifier::AddCert(const std::string& pem, X509_STORE* store) {
  BIO* bio = BIO_new_mem_buf(pem.c_str(), pem.size());
  if (bio == nullptr) {
    RTC_LOG(LS_ERROR) << "BIO_new_mem_buf failed";
    return false;
  }
  X509* cert = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr);
  if (cert == nullptr) {
    BIO_free(bio);
    RTC_LOG(LS_ERROR) << "PEM_read_bio_X509 failed";
    return false;
  }
  int r = X509_STORE_add_cert(store, cert);
  if (r == 0) {
    X509_free(cert);
    BIO_free(bio);
    RTC_LOG(LS_ERROR) << "X509_STORE_add_cert failed";
    return false;
  }
  X509_free(cert);
  BIO_free(bio);
  return true;
}

// rtc_base/openssl_utility.cc を今回の用途に合わせたもの
bool SSLVerifier::LoadBuiltinSSLRootCertificates(X509_STORE* store) {
  int count_of_added_certs = 0;
  for (size_t i = 0;
       i < sizeof(kSSLCertCertificateList) / sizeof(kSSLCertCertificateList[0]);
       i++) {
    const unsigned char* cert_buffer = kSSLCertCertificateList[i];
    size_t cert_buffer_len = kSSLCertCertificateSizeList[i];
    X509* cert = d2i_X509(nullptr, &cert_buffer,
                          cert_buffer_len);  // NOLINT
    if (cert) {
      int return_value = X509_STORE_add_cert(store, cert);
      if (return_value == 0) {
        RTC_LOG(LS_WARNING) << "Unable to add certificate.";
      } else {
        count_of_added_certs++;
      }
      X509_free(cert);
    }
  }
  return count_of_added_certs > 0;
}

bool SSLVerifier::VerifyX509(X509* x509) {
  {
    char subject_name[256];
    X509_NAME_oneline(X509_get_subject_name(x509), subject_name, 256);
    RTC_LOG(LS_INFO) << "Verifying " << subject_name;
  }

  X509_STORE* store = nullptr;
  X509_STORE_CTX* ctx = nullptr;

  struct Guard {
    std::function<void()> f;
    Guard(std::function<void()> f) : f(std::move(f)) {}
    ~Guard() { f(); }
  };
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

  // WebRTC が用意しているルート証明書の設定
  LoadBuiltinSSLRootCertificates(store);
  // デフォルト証明書のパスの設定
  X509_STORE_set_default_paths(store);

  ctx = X509_STORE_CTX_new();
  if (ctx == nullptr) {
    RTC_LOG(LS_ERROR) << "X509_STORE_CTX_new failed";
    return false;
  }
  int r;
  r = X509_STORE_CTX_init(ctx, store, x509, nullptr);
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
