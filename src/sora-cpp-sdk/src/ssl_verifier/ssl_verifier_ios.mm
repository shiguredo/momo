#include "sora/ssl_verifier.h"

#include <vector>

#import <CoreFoundation/CoreFoundation.h>
#import <Security/Security.h>

// OpenSSL
#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/pem.h>
#include <openssl/x509.h>

// WebRTC
#include <rtc_base/logging.h>

#include "ssl_verifier_util.h"

namespace sora {
namespace {

// X509 を DER にエンコードして SecCertificateRef を作る
// 失敗時は nullptr を返す
SecCertificateRef CreateSecCertificate(X509* cert) {
  unsigned char* der = nullptr;
  int len = i2d_X509(cert, &der);
  if (len <= 0 || der == nullptr) {
    // i2d_X509 失敗時に積まれたエラーをクリア
    ERR_get_error();
    return nullptr;
  }
  CFDataRef data = CFDataCreate(nullptr, der, len);
  OPENSSL_free(der);
  if (data == nullptr) {
    return nullptr;
  }
  SecCertificateRef sec_cert = SecCertificateCreateWithData(nullptr, data);
  CFRelease(data);
  return sec_cert;
}

// ca_cert の PEM 文字列から SecCertificateRef の配列を作る
// PEM は複数の CERTIFICATE ブロックを含みうる
// 1 件も取れなければ空を返す
std::vector<SecCertificateRef> LoadCACertsFromPEM(const std::string& pem) {
  std::vector<SecCertificateRef> result;
  std::vector<X509*> certs = ParsePEMCerts(pem);
  for (X509* cert : certs) {
    SecCertificateRef sec_cert = CreateSecCertificate(cert);
    X509_free(cert);
    if (sec_cert == nullptr) {
      RTC_LOG(LS_WARNING) << "VerifyX509: failed to convert ca_cert PEM entry "
                             "to SecCertificateRef";
      continue;
    }
    result.push_back(sec_cert);
  }
  return result;
}

}  // namespace

bool SSLVerifier::VerifyX509(X509* x509,
                             STACK_OF(X509) * chain,
                             const std::optional<std::string>& ca_cert) {
  DumpX509CertificateInfo(x509, chain);

  // 引数の X509 と chain を SecCertificateRef の配列に変換する
  std::vector<SecCertificateRef> sec_certs;
  Guard certs_guard([&sec_certs]() {
    for (SecCertificateRef c : sec_certs) {
      CFRelease(c);
    }
  });

  SecCertificateRef leaf = CreateSecCertificate(x509);
  if (leaf == nullptr) {
    RTC_LOG(LS_ERROR) << "VerifyX509: failed to convert leaf certificate";
    return false;
  }
  sec_certs.push_back(leaf);

  if (chain != nullptr) {
    size_t n = sk_X509_num(chain);
    for (size_t i = 0; i < n; i++) {
      SecCertificateRef mid = CreateSecCertificate(sk_X509_value(chain, i));
      if (mid == nullptr) {
        // 中間証明書 1 個の変換失敗は続行する
        // chain が構築できなければ最終的に SecTrust 側で失敗するため
        RTC_LOG(LS_WARNING)
            << "VerifyX509: failed to convert intermediate certificate: index="
            << i;
        continue;
      }
      sec_certs.push_back(mid);
    }
  }

  // CFArrayCreate に渡すため const void* の配列に変換する
  std::vector<const void*> raw_ptrs;
  raw_ptrs.reserve(sec_certs.size());
  for (auto c : sec_certs) {
    raw_ptrs.push_back(static_cast<const void*>(c));
  }
  CFArrayRef cert_array = CFArrayCreate(
      nullptr, raw_ptrs.data(), raw_ptrs.size(), &kCFTypeArrayCallBacks);
  if (cert_array == nullptr) {
    RTC_LOG(LS_ERROR) << "VerifyX509: CFArrayCreate failed";
    return false;
  }
  Guard cert_array_guard([cert_array]() { CFRelease(cert_array); });

  SecPolicyRef policy = SecPolicyCreateBasicX509();
  if (policy == nullptr) {
    RTC_LOG(LS_ERROR) << "VerifyX509: SecPolicyCreateBasicX509 failed";
    return false;
  }
  Guard policy_guard([policy]() { CFRelease(policy); });

  SecTrustRef trust = nullptr;
  OSStatus status = SecTrustCreateWithCertificates(cert_array, policy, &trust);
  if (status != errSecSuccess || trust == nullptr) {
    RTC_LOG(LS_ERROR)
        << "VerifyX509: SecTrustCreateWithCertificates failed: status="
        << status;
    return false;
  }
  Guard trust_guard([trust]() { CFRelease(trust); });

  if (ca_cert) {
    // ca_cert 指定時: 指定 PEM のみを anchor にし、system CA と混ぜない
    std::vector<SecCertificateRef> anchors = LoadCACertsFromPEM(*ca_cert);
    Guard anchors_guard([&anchors]() {
      for (SecCertificateRef c : anchors) {
        CFRelease(c);
      }
    });
    if (anchors.empty()) {
      RTC_LOG(LS_ERROR)
          << "VerifyX509: no anchors loaded from ca_cert: ca_cert_length="
          << ca_cert->size();
      return false;
    }
    std::vector<const void*> anchor_ptrs;
    anchor_ptrs.reserve(anchors.size());
    for (auto c : anchors) {
      anchor_ptrs.push_back(static_cast<const void*>(c));
    }
    CFArrayRef anchor_array =
        CFArrayCreate(nullptr, anchor_ptrs.data(), anchor_ptrs.size(),
                      &kCFTypeArrayCallBacks);
    if (anchor_array == nullptr) {
      RTC_LOG(LS_ERROR) << "VerifyX509: CFArrayCreate for anchors failed";
      return false;
    }
    Guard anchor_array_guard([anchor_array]() { CFRelease(anchor_array); });

    status = SecTrustSetAnchorCertificates(trust, anchor_array);
    if (status != errSecSuccess) {
      RTC_LOG(LS_ERROR)
          << "VerifyX509: SecTrustSetAnchorCertificates failed: status="
          << status;
      return false;
    }
    status = SecTrustSetAnchorCertificatesOnly(trust, true);
    if (status != errSecSuccess) {
      RTC_LOG(LS_ERROR)
          << "VerifyX509: SecTrustSetAnchorCertificatesOnly failed: status="
          << status;
      return false;
    }
  }
  // ca_cert 未指定時: SecTrust は system trust store をそのまま使う

  CFErrorRef error = nullptr;
  bool verified = SecTrustEvaluateWithError(trust, &error);
  if (!verified) {
    if (error != nullptr) {
      CFStringRef desc = CFErrorCopyDescription(error);
      char buf[512] = {0};
      if (desc != nullptr) {
        if (!CFStringGetCString(desc, buf, sizeof(buf),
                                kCFStringEncodingUTF8)) {
          RTC_LOG(LS_WARNING)
              << "VerifyX509: SecTrustEvaluateWithError failed: "
                 "unable to copy error description";
        } else {
          RTC_LOG(LS_WARNING)
              << "VerifyX509: SecTrustEvaluateWithError failed: " << buf;
        }
        CFRelease(desc);
      }
      CFRelease(error);
    } else {
      RTC_LOG(LS_WARNING)
          << "VerifyX509: SecTrustEvaluateWithError failed: no error info";
    }
    return false;
  }
  RTC_LOG(LS_INFO) << "VerifyX509: SecTrustEvaluateWithError succeeded";
  return true;
}

}  // namespace sora
