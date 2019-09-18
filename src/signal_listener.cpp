#include "signal_listener.h"

#include <algorithm>
#include <csignal>

SignalListener::SignalListener() {
  SignalManager::add(this);
}

SignalListener::~SignalListener() {
  SignalManager::remove(this);
}

std::vector<SignalListener*> SignalManager::_listeners;

void SignalManager::sigintHandler(int signum) {
  for (SignalListener* listener : _listeners) {
    listener->OnSignal(signum);
  }
}

void SignalManager::init() {
  signal(SIGINT, SignalManager::sigintHandler);
}

void SignalManager::add(SignalListener* listener) {
  _listeners.push_back(listener);
}

void SignalManager::remove(SignalListener* listener) {
  _listeners.erase(std::remove(_listeners.begin(), _listeners.end(), listener),
                   _listeners.end());
}