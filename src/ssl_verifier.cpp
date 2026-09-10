#include "ssl_verifier.h"

// webrtc
#include <rtc_base/logging.h>

// openssl
#include <openssl/x509v3.h>

#if defined(_WIN32)
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#endif

#include "sora/ssl_verifier.h"

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

bool SSLVerifier::VerifyX509(X509* x509,
                             STACK_OF(X509) * chain,
                             const std::string& host,
                             const std::optional<std::string>& ca_cert) {
  // 信頼ストアの構築とチェーン検証は sora-cpp-sdk 側に委譲する
  if (!sora::SSLVerifier::VerifyX509(x509, chain, ca_cert)) {
    return false;
  }

  if (host.empty()) {
    return true;
  }
  return VerifyHostname(x509, host);
}
