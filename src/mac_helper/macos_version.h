#ifndef MACOS_VERSION_H_
#define MACOS_VERSION_H_

#include <string>

class MacosVersion {
 public:
  static std::string GetOSName();
  static std::string GetOSVersion();
};

#endif  // MACOS_VERSION_H_
