#ifndef WATCH_DOG_H_
#define WATCH_DOG_H_

#include <functional>

// Boost
#include <boost/asio.hpp>

/*
タイムアウトを監視するためのクラス。

enable() を呼び出した後、一定時間が経過するとコールバック関数が呼ばれる。
コールバックが発生する時 WatchDog は無効になるので、必要であれば再度 enable() や reset() を呼び出すこと。
マルチスレッド下では動作しないので注意。
*/
class WatchDog {
 public:
  WatchDog(boost::asio::io_context& ioc, std::function<void()> callback);
  void enable(int timeout);
  void disable();
  void reset();

 private:
  int timeout_;
  boost::asio::deadline_timer timer_;
  std::function<void()> callback_;
};

#endif
