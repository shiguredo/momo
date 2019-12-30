#ifndef MACOS_VERSION_H_INCLUDED
#define MACOS_VERSION_H_INCLUDED

#include <string>

class MacosVersion {
 public:
  static std::string GetOSName();
  static std::string GetOSVersion();
};

#endif // MACOS_VERSION_H_INCLUDED
