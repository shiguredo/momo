#ifndef SERIAL_DATA_CHANNEL_H_
#define SERIAL_DATA_CHANNEL_H_

#include "api/data_channel_interface.h"

#include "serial_data_manager.h"

class SerialDataManager;

class SerialDataChannel : public webrtc::DataChannelObserver {
 public:
  SerialDataChannel(
      SerialDataManager* serial_data_manager,
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel);

  void OnStateChange() override;
  void OnMessage(const webrtc::DataBuffer& buffer) override;
  void OnBufferedAmountChange(uint64_t previous_amount) override {}

  void send(uint8_t* data, size_t length);

 private:
  SerialDataManager* serial_data_manager_;
  rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel_;
};

#endif