#ifndef RTC_SSL_VERIFIER
#define RTC_SSL_VERIFIER

// WebRTC
#include <rtc_base/ssl_certificate.h>

class RTCSSLVerifier : public webrtc::SSLCertificateVerifier {
 public:
  RTCSSLVerifier(bool insecure);
  bool VerifyChain(const webrtc::SSLCertChain& chain) override;

 private:
  bool insecure_;
};

#endif
