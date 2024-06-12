#ifndef VPL_UTILS_H_
#define VPL_UTILS_H_

#define VPL_CHECK_RESULT(P, X, ERR)                 \
  {                                                 \
    if ((X) > (P)) {                                \
      RTC_LOG(LS_ERROR) << "oneVPL Error: " << ERR; \
      throw ERR;                                    \
    }                                               \
  }

#endif