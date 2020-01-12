#ifndef MOMO_VERSION_H_INCLUDED
#define MOMO_VERSION_H_INCLUDED

#include <string>

class MomoVersion {
 public:
  static std::string GetClientName();
  static std::string GetLibwebrtcName();
  static std::string GetEnvironmentName();
};

#endif  // MOMO_VERSION_H_INCLUDED
