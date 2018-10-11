#ifndef WATCH_DOG_H_
#define WATCH_DOG_H_

#include <mutex>
#include <thread>
#include <atomic>

// タイムアウトを監視するためのクラス。
// enable() を呼び出した後、一定時間が経過するとコールバック関数が呼ばれる。
// コールバック関数は WatchDog のスレッド上で呼ばれるため、必要であれば排他制御を行うこと。
// コールバックが発生する時、WatchDog は無効になるので、必要であれば再度 enable() を呼び出すこと。
class WatchDog
{
public:
  WatchDog(std::function<void()> callback);
  ~WatchDog();
  void enable(int timeout);
  void disable();
  void reset();

private:
  void loop();

  bool _enable;
  bool _running;
  std::function<void()> _callback;
  std::chrono::system_clock::time_point _abs_timeout;
  std::chrono::system_clock::duration _timeout;

  std::mutex _mutex;
  std::condition_variable _condition;
  std::unique_ptr<std::thread> _thread;
};
#endif
