#ifndef SSL_VERIFIER_H_
#define SSL_VERIFIER_H_

#include <optional>
#include <string>

// openssl
#include <openssl/ssl.h>

// 自前で SSL の証明書検証を行うためのクラス。
// チェーン検証は sora::SSLVerifier に委譲し、WSS 向けのホスト名検証だけをここで行う。
class SSLVerifier {
 public:
  // host が空の場合はチェーン検証のみを行う
  static bool VerifyX509(X509* x509,
                         STACK_OF(X509) * chain,
                         const std::string& host,
                         const std::optional<std::string>& ca_cert);
};

#endif  // SSL_VERIFIER_H_
