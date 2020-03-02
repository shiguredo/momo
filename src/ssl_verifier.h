#ifndef SSL_VERIFIER_H_INCLUDED
#define SSL_VERIFIER_H_INCLUDED

#include <string>

// openssl
#include <openssl/ssl.h>

// 自前で SSL の証明書検証を行うためのクラス
class SSLVerifier {
 public:
  static bool VerifyHost(const std::string& hostname, const std::string& port);
  static bool VerifyX509(X509* x509);

 private:
  // ルート証明書の追加
  static bool AddCert(const std::string& pem, X509_STORE* store);
};

#endif  // SSL_VERIFIER_H_INCLUDED
