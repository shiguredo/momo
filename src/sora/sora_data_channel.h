#ifndef SORA_DATA_CHANNEL_H_
#define SORA_DATA_CHANNEL_H_

#include <map>
#include <memory>
#include <set>

// Boost
#include <boost/asio.hpp>

// WebRTC
#include <api/data_channel_interface.h>
#include <rtc_base/synchronization/mutex.h>

#include "rtc/rtc_data_manager.h"

class SoraDataChannelObserver {
 public:
  ~SoraDataChannelObserver() {}
  virtual void OnStateChange(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) = 0;
  virtual void OnMessage(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel,
      const webrtc::DataBuffer& buffer) = 0;
};

// 複数の DataChannel を纏めてコールバックで受け取るためのクラス
class SoraDataChannel : public RTCDataManager {
  struct Thunk : webrtc::DataChannelObserver,
                 std::enable_shared_from_this<Thunk> {
    SoraDataChannel* p;
    rtc::scoped_refptr<webrtc::DataChannelInterface> dc;
    void OnStateChange() override { p->OnStateChange(shared_from_this()); }
    void OnMessage(const webrtc::DataBuffer& buffer) override {
      p->OnMessage(shared_from_this(), buffer);
    }
    void OnBufferedAmountChange(uint64_t previous_amount) override {
      p->OnBufferedAmountChange(shared_from_this(), previous_amount);
    }
  };

 public:
  SoraDataChannel(SoraDataChannelObserver* observer) : observer_(observer) {}
  bool IsOpen(std::string label) const {
    webrtc::MutexLock lock(&mutex_);
    return labels_.find(label) != labels_.end();
  }
  void Send(std::string label, const webrtc::DataBuffer& data) {
    webrtc::MutexLock lock(&mutex_);
    auto it = labels_.find(label);
    if (it == labels_.end()) {
      return;
    }
    std::string str((const char*)data.data.cdata(),
                    (const char*)data.data.cdata() + data.size());
    RTC_LOG(LS_INFO) << "Send DataChannel label=" << label << " data=" << str;
    auto data_channel = it->second;
    data_channel->Send(data);
  }
  void Close(std::function<void()> on_close) {
    webrtc::MutexLock lock(&mutex_);
    auto it = labels_.find("signaling");
    if (it == labels_.end()) {
      mutex_.Unlock();
      on_close();
      mutex_.Lock();
      return;
    }
    on_close_ = on_close;
    std::string str = R"({"type":"disconnect})";
    RTC_LOG(LS_INFO) << "Send DataChannel label=signaling data=" << str;
    webrtc::DataBuffer data(rtc::CopyOnWriteBuffer(str), false);
    auto data_channel = it->second;
    data_channel->Send(data);
  }

 public:
  void OnDataChannel(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) override {
    webrtc::MutexLock lock(&mutex_);
    std::shared_ptr<Thunk> thunk(new Thunk());
    thunk->p = this;
    thunk->dc = data_channel;
    data_channel->RegisterObserver(thunk.get());
    thunks_.insert(std::make_pair(thunk, data_channel));
    labels_.insert(std::make_pair(data_channel->label(), data_channel));
  }

 private:
  void OnStateChange(std::shared_ptr<Thunk> thunk) {
    webrtc::MutexLock lock(&mutex_);
    auto data_channel = thunks_.at(thunk);
    if (data_channel->state() == webrtc::DataChannelInterface::kClosed) {
      labels_.erase(data_channel->label());
      thunks_.erase(thunk);
    }
    auto observer = observer_;
    auto on_close = std::move(on_close_);
    on_close_ = nullptr;
    auto empty = thunks_.empty();
    mutex_.Unlock();
    observer->OnStateChange(data_channel);
    // すべての Data Channel が閉じたら通知する
    if (on_close != nullptr && empty) {
      on_close();
    }
    mutex_.Lock();
  }

  void OnMessage(std::shared_ptr<Thunk> thunk,
                 const webrtc::DataBuffer& buffer) {
    webrtc::MutexLock lock(&mutex_);
    auto observer = observer_;
    auto data_channel = thunks_.at(thunk);
    mutex_.Unlock();
    observer->OnMessage(data_channel, buffer);
    mutex_.Lock();
  }
  void OnBufferedAmountChange(std::shared_ptr<Thunk> thunk,
                              uint64_t previous_amount) {}

 private:
  std::map<std::shared_ptr<Thunk>,
           rtc::scoped_refptr<webrtc::DataChannelInterface>>
      thunks_;
  std::map<std::string, rtc::scoped_refptr<webrtc::DataChannelInterface>>
      labels_;
  SoraDataChannelObserver* observer_;
  std::function<void()> on_close_;
  mutable webrtc::Mutex mutex_;
};

#endif