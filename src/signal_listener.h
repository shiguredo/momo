#ifndef SIGNAL_LISTENER_H_
#define SIGNAL_LISTENER_H_

#include <vector>

class SignalManager;

class SignalListener
{
public:
  SignalListener();
  ~SignalListener();

  virtual void OnSignal(int signum) = 0;
};

class SignalManager
{
public:
  static void init();
  static void add(SignalListener *listener);
  static void remove(SignalListener *listener);

private:
  static void sigintHandler(int signum);

  static std::vector<SignalListener *> _listeners;
};
#endif