// OpenSSL
#include <openssl/err.h>
#include <openssl/x509.h>

// clang-format off
// Windows CryptoAPI
#include <windows.h>
#include <wincrypt.h>
// clang-format on

// WebRTC
#include <rtc_base/logging.h>

#include "ssl_verifier_util.h"

namespace sora {

bool LoadSystemSSLRootCertificates(X509_STORE* store) {
  // 第 1 引数 hProv は MSDN 仕様どおり NULL を渡す（本引数は使用されない）
  // 第 2 引数 L"ROOT" は Windows の「信頼されたルート証明機関」ストア
  HCERTSTORE h_store = CertOpenSystemStoreW(NULL, L"ROOT");
  if (h_store == NULL) {
    // GetLastError は後続の Win32 呼び出しで上書きされる可能性があるため即時に取得する
    DWORD err = GetLastError();
    RTC_LOG(LS_ERROR) << "LoadSystemSSLRootCertificates: CertOpenSystemStoreW "
                         "failed: last_error="
                      << err;
    return false;
  }
  Guard store_guard([h_store]() { CertCloseStore(h_store, 0); });

  int added = 0;
  // CertEnumCertificatesInStore は次回呼び出しで前回の PCCERT_CONTEXT を
  // 自動解放するため、ループ中の CertFreeCertificateContext は呼ばない
  PCCERT_CONTEXT ctx = nullptr;
  DWORD enum_last_error = CRYPT_E_NOT_FOUND;
  while (true) {
    ctx = CertEnumCertificatesInStore(h_store, ctx);
    if (ctx == nullptr) {
      // ループ本体内の操作が SetLastError() を上書きする可能性があるため、
      // CertEnumCertificatesInStore が nullptr を返した直後にエラー値を保存する
      enum_last_error = GetLastError();
      break;
    }
    // dwCertEncodingType には通常 X509_ASN_ENCODING (0x1) のみが入るが、
    // 将来の CryptoAPI 拡張で他エンコーディングが混ざった場合の防御としてビット判定する
    if ((ctx->dwCertEncodingType & X509_ASN_ENCODING) == 0) {
      continue;
    }
    const unsigned char* p = ctx->pbCertEncoded;
    // d2i_X509 は戻り時点で pbCertEncoded のバイト列のパースを完了しているため、
    // 以降 ctx（および ctx が指すバイト列）の寿命は気にしなくてよい
    X509* cert = d2i_X509(nullptr, &p, static_cast<long>(ctx->cbCertEncoded));
    if (cert == nullptr) {
      // d2i_X509 失敗でエラーキューが積まれるため 1 回取り出してクリアする（TryAddCertToStore と同型）
      ERR_get_error();
      RTC_LOG(LS_WARNING) << "LoadSystemSSLRootCertificates: d2i_X509 failed";
      continue;
    }
    if (TryAddCertToStore(cert, store, "LoadSystemSSLRootCertificates")) {
      ++added;
    }
    X509_free(cert);
  }
  // ループを抜けた時点で ctx == nullptr。MSDN 仕様上 nullptr は「列挙完了」と
  // 「途中エラー」の両方を意味するため、上で保存した enum_last_error で識別する
  if (enum_last_error != CRYPT_E_NOT_FOUND) {
    RTC_LOG(LS_WARNING)
        << "LoadSystemSSLRootCertificates: CertEnumCertificatesInStore ended "
           "abnormally: last_error="
        << enum_last_error;
  }

  if (added == 0) {
    RTC_LOG(LS_ERROR) << "LoadSystemSSLRootCertificates: no certificates "
                         "loaded from Windows ROOT store";
    return false;
  }
  RTC_LOG(LS_INFO) << "LoadSystemSSLRootCertificates: added=" << added;
  return true;
}

}  // namespace sora
