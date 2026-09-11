#ifndef RTC_SSL_VERIFIER
#define RTC_SSL_VERIFIER

#include <optional>
#include <string>

// WebRTC
#include <rtc_base/ssl_certificate.h>

class RTCSSLVerifier : public webrtc::SSLCertificateVerifier {
 public:
  RTCSSLVerifier(bool insecure, const std::optional<std::string>& ca_cert);
  bool VerifyChain(const webrtc::SSLCertChain& chain) override;

 private:
  bool insecure_;
  std::optional<std::string> ca_cert_;
};

#endif
