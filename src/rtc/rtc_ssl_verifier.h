#ifndef RTC_SSL_VERIFIER
#define RTC_SSL_VERIFIER

// WebRTC
#include <rtc_base/ssl_certificate.h>

class RTCSSLVerifier : public webrtc::SSLCertificateVerifier {
 public:
  RTCSSLVerifier(bool insecure);
  bool Verify(const webrtc::SSLCertificate& certificate) override;

 private:
  bool insecure_;
};

#endif
