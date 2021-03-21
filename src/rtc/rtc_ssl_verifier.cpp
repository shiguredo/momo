#include "rtc_ssl_verifier.h"

// WebRTC
#include <rtc_base/boringssl_certificate.h>

#include "ssl_verifier.h"

RTCSSLVerifier::RTCSSLVerifier(bool insecure) : insecure_(insecure) {}

bool RTCSSLVerifier::Verify(const rtc::SSLCertificate& certificate) {
  // insecure の場合は証明書をチェックしない
  if (insecure_) {
    return true;
  }
  CRYPTO_BUFFER* cert = static_cast<const rtc::BoringSSLCertificate&>(certificate).cert_buffer();
  bssl::UniquePtr<X509> x509(X509_parse_from_buffer(cert));
  if (!x509) {
    return false;
  }
  return SSLVerifier::VerifyX509(x509.get());
}
