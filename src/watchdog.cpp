#include "watchdog.h"

#include <iostream>

WatchDog::WatchDog(boost::asio::io_context& ioc, std::function<void()> callback)
    : timer_(ioc), callback_(callback), timeout_(0) {}

void WatchDog::Enable(int timeout) {
  timeout_ = timeout;
  timer_.cancel();
  timer_.expires_from_now(boost::posix_time::seconds(timeout));
  timer_.async_wait([this](const boost::system::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) {
      return;
    }

    callback_();
  });
}

void WatchDog::Disable() {
  timer_.cancel();
}

void WatchDog::Reset() {
  Enable(timeout_);
}
