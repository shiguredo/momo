#ifndef SSL_VERIFIER_H_INCLUDED
#define SSL_VERIFIER_H_INCLUDED

#include <string>

// openssl
#include <openssl/ssl.h>

// 自前で SSL の証明書検証を行うためのクラス
class SSLVerifier {
 public:
  static bool VerifyX509(X509* x509);

 private:
  // PEM 形式のルート証明書を追加する
  static bool AddCert(const std::string& pem, X509_STORE* store);
  // WebRTC の組み込みルート証明書を追加する
  static bool LoadBuiltinSSLRootCertificates(X509_STORE* store);
};

#endif  // SSL_VERIFIER_H_INCLUDED
