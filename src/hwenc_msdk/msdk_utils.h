#ifndef MSDK_UTILS_H_
#define MSDK_UTILS_H_

#define MSDK_CHECK_RESULT(P, X, ERR)                         \
  {                                                          \
    if ((X) > (P)) {                                         \
      RTC_LOG(LS_ERROR) << "Intel Media SDK Error: " << ERR; \
      throw ERR;                                             \
    }                                                        \
  }

#endif