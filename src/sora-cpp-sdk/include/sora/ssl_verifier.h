#ifndef SORA_SSL_VERIFIER_H_
#define SORA_SSL_VERIFIER_H_

#include <optional>
#include <string>

// openssl
#include <openssl/ssl.h>
#include <openssl/stack.h>

namespace sora {

// 自前で SSL の証明書検証を行うためのクラス
class SSLVerifier {
 public:
  static bool VerifyX509(X509* x509,
                         STACK_OF(X509) * chain,
                         const std::optional<std::string>& ca_cert);
};

}  // namespace sora

#endif
