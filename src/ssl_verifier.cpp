#include "ssl_verifier.h"

// webrtc
#include <rtc_base/logging.h>
#include <rtc_base/openssl_utility.h>
#include <rtc_base/ssl_roots.h>

// openssl
#include <openssl/x509v3.h>

#if defined(_WIN32)
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#endif

// boost
#include <boost/asio/connect.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>

const char isrg_root[] = R"(
# Issuer: CN=ISRG Root X1 O=Internet Security Research Group
# Subject: CN=ISRG Root X1 O=Internet Security Research Group
# Label: "ISRG Root X1"
# Serial: 172886928669790476064670243504169061120
# MD5 Fingerprint: 0c:d2:f9:e0:da:17:73:e9:ed:86:4d:a5:e3:70:e7:4e
# SHA1 Fingerprint: ca:bd:2a:79:a1:07:6a:31:f2:1d:25:36:35:cb:03:9d:43:29:a5:e8
# SHA256 Fingerprint: 96:bc:ec:06:26:49:76:f3:74:60:77:9a:cf:28:c5:a7:cf:e8:a3:c0:aa:e1:1a:8f:fc:ee:05:c0:bd:df:08:c6
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)";

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

namespace {

// 正規化済みホスト名が IP アドレスかどうかを判定し、証明書の SAN / CN と照合する
bool VerifyHostname(X509* x509, const std::string& host) {
  struct in6_addr addr6;
  if (inet_pton(AF_INET6, host.c_str(), &addr6) == 1) {
    int r = X509_check_ip(x509, reinterpret_cast<const unsigned char*>(&addr6),
                          sizeof(addr6), 0);
    if (r != 1) {
      RTC_LOG(LS_ERROR) << "X509_check_ip failed: host=" << host
                        << " result=" << r;
      return false;
    }
    return true;
  }

  struct in_addr addr4;
  if (inet_pton(AF_INET, host.c_str(), &addr4) == 1) {
    int r = X509_check_ip(x509, reinterpret_cast<const unsigned char*>(&addr4),
                          sizeof(addr4), 0);
    if (r != 1) {
      RTC_LOG(LS_ERROR) << "X509_check_ip failed: host=" << host
                        << " result=" << r;
      return false;
    }
    return true;
  }

  int r = X509_check_host(x509, host.c_str(), host.size(), 0, nullptr);
  if (r != 1) {
    RTC_LOG(LS_ERROR) << "X509_check_host failed: host=" << host
                      << " result=" << r;
    return false;
  }
  return true;
}

}  // namespace

bool SSLVerifier::VerifyX509(X509* x509, STACK_OF(X509) * chain) {
  return VerifyX509(x509, chain, "");
}

bool SSLVerifier::VerifyX509(X509* x509,
                             STACK_OF(X509) * chain,
                             const std::string& host) {
  {
    char data[256];
    RTC_LOG(LS_INFO) << "cert:";
    X509_NAME_oneline(X509_get_subject_name(x509), data, sizeof(data));
    RTC_LOG(LS_INFO) << "  subject = " << data;
    X509_NAME_oneline(X509_get_issuer_name(x509), data, sizeof(data));
    RTC_LOG(LS_INFO) << "  issuer  = " << data;

    if (chain != nullptr) {
      int n = sk_X509_num(chain);
      for (int i = 0; i < n; i++) {
        X509* x = sk_X509_value(chain, i);
        RTC_LOG(LS_INFO) << "chain[" << i << "]:";
        X509_NAME_oneline(X509_get_subject_name(x), data, sizeof(data));
        RTC_LOG(LS_INFO) << "  subject = " << data;
        X509_NAME_oneline(X509_get_issuer_name(x), data, sizeof(data));
        RTC_LOG(LS_INFO) << "  issuer  = " << data;
      }
    }
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
  int r;
  r = X509_STORE_set_flags(store, X509_V_FLAG_TRUSTED_FIRST);
  if (r == 0) {
    RTC_LOG(LS_ERROR) << "X509_STORE_set_flags failed";
    return false;
  }

  // Let's Encrypt のルート証明書を追加
  if (!AddCert(isrg_root, store)) {
    return false;
  }

  // WebRTC が用意しているルート証明書の設定
  LoadBuiltinSSLRootCertificates(store);
  // デフォルト証明書のパスの設定
  X509_STORE_set_default_paths(store);
  RTC_LOG(LS_INFO) << "default cert file: " << X509_get_default_cert_file();

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

  if (host.empty()) {
    return true;
  }
  return VerifyHostname(x509, host);
}
