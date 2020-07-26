#ifndef MOMO_VERSION_H_
#define MOMO_VERSION_H_

#include <string>

class MomoVersion {
 public:
  static std::string GetClientName();
  static std::string GetLibwebrtcName();
  static std::string GetEnvironmentName();
};

#endif  // MOMO_VERSION_H_
