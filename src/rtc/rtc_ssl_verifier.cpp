#include "rtc_ssl_verifier.h"

// WebRTC
#include <rtc_base/openssl_certificate.h>

#include "ssl_verifier.h"

RTCSSLVerifier::RTCSSLVerifier(bool insecure) : insecure_(insecure) {}

bool RTCSSLVerifier::Verify(const rtc::SSLCertificate& certificate) {
  // insecure の場合は証明書をチェックしない
  if (insecure_) {
    return true;
  }
  return SSLVerifier::VerifyX509(
      static_cast<const rtc::OpenSSLCertificate&>(certificate).x509());
}
