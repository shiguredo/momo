#include "momo_version.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// boost
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

#if defined(__APPLE__) || defined(__linux__)
#include <sys/utsname.h>
#endif
#if defined(__APPLE__)
#include "mac_helper/macos_version.h"
#endif

// バージョンやコミットハッシュ情報
// 通常は外から渡すが、渡されていなかった場合の対応
#ifndef MOMO_VERSION
#define MOMO_VERSION "internal-build"
#endif

#ifndef MOMO_COMMIT_SHORT
#define MOMO_COMMIT_SHORT "unknown"
#endif

#define MOMO_NAME \
  "WebRTC Native Client Momo " MOMO_VERSION " (" MOMO_COMMIT_SHORT ")"

#if defined(WEBRTC_READABLE_VERSION) && defined(WEBRTC_COMMIT_SHORT) && \
    defined(WEBRTC_BUILD_VERSION)

#define LIBWEBRTC_NAME                                                 \
  "Shiguredo-Build " WEBRTC_READABLE_VERSION " (" WEBRTC_BUILD_VERSION \
  " " WEBRTC_COMMIT_SHORT ")"

#else

#define LIBWEBRTC_NAME "WebRTC custom build"

#endif

std::string MomoVersion::GetClientName() {
  return MOMO_NAME;
}

std::string MomoVersion::GetLibwebrtcName() {
  return LIBWEBRTC_NAME;
}

std::string MomoVersion::GetEnvironmentName() {
  std::string environment = "Unknown Environment";

#if defined(__APPLE__) || defined(__linux__)
  std::string arch = "unknown arch";
  std::string os = "Unknown OS";
  std::string info = "";

  utsname u;
  int r = uname(&u);
  if (r == 0) {
    arch = u.machine;
  }

#if defined(__APPLE__)
  os = MacosVersion::GetOSName() + " " + MacosVersion::GetOSVersion();
#else
  // /etc/os-release ファイルを読んで PRETTY_NAME を利用する

  // /etc/os-release は以下のような内容になっているので、これを適当にパースする
  /*
      $ docker run -it --rm ubuntu cat /etc/os-release
      NAME="Ubuntu"
      VERSION="18.04.3 LTS (Bionic Beaver)"
      ID=ubuntu
      ID_LIKE=debian
      PRETTY_NAME="Ubuntu 18.04.3 LTS"
      VERSION_ID="18.04"
      HOME_URL="https://www.ubuntu.com/"
      SUPPORT_URL="https://help.ubuntu.com/"
      BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
      PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
      VERSION_CODENAME=bionic
      UBUNTU_CODENAME=bionic
    */
  // 行ごとに分けたデータを取得
  std::vector<std::string> lines;
  {
    std::stringstream ss;
    std::ifstream fin("/etc/os-release");
    ss << fin.rdbuf();
    std::string content = ss.str();
    boost::algorithm::split(lines, ss.str(), boost::is_any_of("\n"));
  }
  const std::string PRETTY_NAME = "PRETTY_NAME=";
  for (auto& line : lines) {
    // 先頭が PRETTY_NAME= の行を探す
    if (line.find(PRETTY_NAME) != 0) {
      continue;
    }
    // PRETTY_NAME= 以降のデータを取り出す
    os = line.substr(PRETTY_NAME.size());
    // 左右の " を除ける（in-place バージョン）
    boost::algorithm::trim_if(os, [](char c) { return c == '"'; });
    break;
  }

#if USE_JETSON_ENCODER
  // Jetson Nano の場合、更に詳細な情報を取得する

  // nvidia-l4t-core のバージョンを拾う
  // $ dpkg-query --show nvidia-l4t-core
  // で取得できるが、外部コマンドは出来るだけ使いたくないので、
  // /var/lib/dpkg/status から該当行を探す

  std::string content;
  {
    std::stringstream ss;
    std::ifstream fin("/var/lib/dpkg/status");
    ss << fin.rdbuf();
    content = ss.str();
  }
  std::string l4t_core_version = "unknown";
  auto pos = content.find("Package: nvidia-l4t-core");
  if (pos != std::string::npos) {
    const std::string VERSION = "Version: ";
    auto pos2 = content.find(VERSION, pos);
    if (pos2 != std::string::npos) {
      pos2 += VERSION.size();
      auto pos3 = content.find("\n", pos2);
      l4t_core_version = pos3 == std::string::npos
                             ? content.substr(pos2)
                             : content.substr(pos2, pos3 - pos2);
    }
  }

  info = " (nvidia-l4t-core " + l4t_core_version + ")";

#endif

#endif

  environment = "[" + arch + "] " + os + info;
#endif

  return environment;
}
