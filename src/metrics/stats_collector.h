#ifndef STATS_COLLECTOR_H_
#define STATS_COLLECTOR_H_

#include "rtc/rtc_connection.h"

class StatsCollector {
 public:
  virtual void GetStats(
      std::function<
          void(const rtc::scoped_refptr<const webrtc::RTCStatsReport>&)>
          callback) = 0;
};

#endif