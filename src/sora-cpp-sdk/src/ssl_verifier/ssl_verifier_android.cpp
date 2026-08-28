#include <dirent.h>
#include <errno.h>
#include <cstdio>

#include <string>
#include <vector>

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

// 証明書ファイルを読み込んで X509* を返す
// DER 形式を先に試し、失敗した場合は PEM 形式も試す。
// どちらも失敗した場合は nullptr を返し、エラー内容を WARNING ログに出力する
// 戻り値は呼び出し元で X509_free すること
X509* ReadCertFile(const std::string& path) {
  FILE* fp = fopen(path.c_str(), "rb");
  if (fp == nullptr) {
    RTC_LOG(LS_WARNING) << "LoadSystemSSLRootCertificates: fopen failed: path="
                        << path;
    return nullptr;
  }
  if (fseek(fp, 0, SEEK_END) != 0) {
    int e = errno;
    fclose(fp);
    RTC_LOG(LS_WARNING) << "LoadSystemSSLRootCertificates: fseek failed: path="
                        << path << " errno=" << e;
    return nullptr;
  }
  long file_size = ftell(fp);
  if (file_size == -1L) {
    fclose(fp);
    RTC_LOG(LS_WARNING) << "LoadSystemSSLRootCertificates: ftell failed: path="
                        << path;
    return nullptr;
  }
  if (fseek(fp, 0, SEEK_SET) != 0) {
    int e = errno;
    fclose(fp);
    RTC_LOG(LS_WARNING) << "LoadSystemSSLRootCertificates: fseek failed: path="
                        << path << " errno=" << e;
    return nullptr;
  }
  if (file_size <= 0) {
    fclose(fp);
    RTC_LOG(LS_WARNING)
        << "LoadSystemSSLRootCertificates: empty or unreadable file: path="
        << path;
    return nullptr;
  }
  std::vector<unsigned char> buf(file_size);
  if (fread(buf.data(), 1, file_size, fp) != static_cast<size_t>(file_size)) {
    fclose(fp);
    RTC_LOG(LS_WARNING) << "LoadSystemSSLRootCertificates: fread failed: path="
                        << path;
    return nullptr;
  }
  if (fclose(fp) != 0) {
    int e = errno;
    RTC_LOG(LS_WARNING) << "LoadSystemSSLRootCertificates: fclose failed: path="
                        << path << " errno=" << e;
  }

  // DER 形式でパースを試みる
  const unsigned char* p = buf.data();
  X509* cert = d2i_X509(nullptr, &p, static_cast<long>(file_size));
  if (cert != nullptr) {
    return cert;
  }
  // DER 失敗時はエラーキューをクリアして PEM を試す
  ERR_get_error();

  // PEM 形式でパースを試みる
  BIO* bio = BIO_new_mem_buf(buf.data(), file_size);
  if (bio == nullptr) {
    ERR_get_error();
    RTC_LOG(LS_WARNING)
        << "LoadSystemSSLRootCertificates: BIO_new_mem_buf failed: path="
        << path;
    return nullptr;
  }
  cert = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr);
  BIO_free(bio);
  if (cert == nullptr) {
    ERR_get_error();
    RTC_LOG(LS_WARNING) << "LoadSystemSSLRootCertificates: both d2i_X509 and "
                           "PEM_read_bio_X509 failed: path="
                        << path;
    return nullptr;
  }
  return cert;
}

// 単一ディレクトリを走査してストアに追加、追加件数を返す。
// opendir 失敗時は errno == ENOENT なら無音で 0 （Android バージョンで片方の経路が無いケース）、
// それ以外は WARNING を出して 0 を返す
int LoadFromDir(X509_STORE* store, const char* dir_path) {
  DIR* dir = opendir(dir_path);
  if (dir == nullptr) {
    int e = errno;
    if (e != ENOENT) {
      RTC_LOG(LS_WARNING)
          << "LoadSystemSSLRootCertificates: opendir failed: path=" << dir_path
          << " errno=" << e;
    }
    return 0;
  }
  Guard dir_guard([dir]() { closedir(dir); });

  int added = 0;
  struct dirent* entry;
  while (true) {
    errno = 0;
    entry = readdir(dir);
    if (entry == nullptr) {
      if (errno != 0) {
        RTC_LOG(LS_WARNING)
            << "LoadSystemSSLRootCertificates: readdir error: path=" << dir_path
            << " errno=" << errno;
      }
      break;
    }
    if (entry->d_name[0] == '.') {
      // AOSP CA ファイルは <subject_hash>.<n> 命名でドット始まりを含まないため
      // "." / ".." およびドット始まりの隠しファイルは対象外で安全
      continue;
    }
    std::string path = std::string(dir_path) + "/" + entry->d_name;
    X509* cert = ReadCertFile(path);
    if (cert == nullptr) {
      continue;
    }
    if (TryAddCertToStore(cert, store, path.c_str())) {
      ++added;
    }
    X509_free(cert);
  }
  return added;
}

}  // namespace

bool LoadSystemSSLRootCertificates(X509_STORE* store) {
  // Conscrypt Mainline module 経由の更新可能な CA ストア（Android 14 以降で提供）を優先
  int added_apex = LoadFromDir(store, "/apex/com.android.conscrypt/cacerts");
  // AOSP 標準の system パス（Android 10-13 の主要ストア、Android 14+ でも残る）
  int added_system = LoadFromDir(store, "/system/etc/security/cacerts");
  int added = added_apex + added_system;

  if (added == 0) {
    RTC_LOG(LS_ERROR)
        << "LoadSystemSSLRootCertificates: no certificates loaded: added=0"
        << " apex=" << added_apex << " system=" << added_system;
    return false;
  }
  RTC_LOG(LS_INFO) << "LoadSystemSSLRootCertificates: added=" << added
                   << ", apex=" << added_apex << ", system=" << added_system;
  return true;
}

}  // namespace sora
