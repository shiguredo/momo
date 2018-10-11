#include "watchdog.h"
#include <iostream>

WatchDog::WatchDog(std::function<void()> callback) : _callback(callback) {
  std::unique_lock<std::mutex> lock(_mutex);
  _enable = false;
  _running = true;
  _thread.reset(new std::thread(&WatchDog::loop, this));
}

WatchDog::~WatchDog() {
  {
    std::unique_lock<std::mutex> lock(_mutex);
    _running = false;
    _enable = false;
    _condition.notify_all();
  }
  _thread->join();
}

void WatchDog::enable(int timeout)
{
  std::unique_lock<std::mutex> lock(_mutex);
  _timeout = std::chrono::seconds(timeout);
  _abs_timeout = std::chrono::system_clock::now() + _timeout;
  _enable = true;
  _condition.notify_all();
}

void WatchDog::disable()
{
  std::unique_lock<std::mutex> lock(_mutex);
  _enable = false;
  _condition.notify_all();
}

void WatchDog::reset()
{
  std::unique_lock<std::mutex> lock(_mutex);
  _abs_timeout = std::chrono::system_clock::now() + _timeout;
  _condition.notify_all();
}

void WatchDog::loop()
{
  while (true)
  {
    std::function<void()> callback;
    {
      std::unique_lock<std::mutex> lock(_mutex);
      // 有効になるまで待つ
      while (_running && !_enable)
      {
        _condition.wait(lock);
      }

      // 終了
      if (!_running)
      {
        return;
      }

      // タイムアウトか、disable されるまで待つ
      while (_running && _enable && std::chrono::system_clock::now() < _abs_timeout)
      {
        _condition.wait_until(lock, _abs_timeout);
      }

      // 終了
      if (!_running)
      {
        return;
      }

      // タイムアウトが発生したので、コールバック呼び出し
      if (_enable && std::chrono::system_clock::now() >= _abs_timeout)
      {
        // ここで呼び出すとロックを取ったままになってしまうので、ロックを抜けてから呼び出す
        callback = _callback;
        _enable = false;
      }
    }
    if (callback)
    {
      callback();
    }
  }
}
