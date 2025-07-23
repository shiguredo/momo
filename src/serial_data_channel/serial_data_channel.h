#ifndef SERIAL_DATA_CHANNEL_H_
#define SERIAL_DATA_CHANNEL_H_

// WebRTC
#include <api/data_channel_interface.h>

#include "serial_data_manager.h"

class SerialDataManager;

class SerialDataChannel : public webrtc::DataChannelObserver {
 public:
  SerialDataChannel(
      SerialDataManager* serial_data_manager,
      webrtc::scoped_refptr<webrtc::DataChannelInterface> data_channel);
  ~SerialDataChannel();

  void Send(uint8_t* data, size_t length);

  void OnStateChange() override;
  void OnMessage(const webrtc::DataBuffer& buffer) override;
  void OnBufferedAmountChange(uint64_t previous_amount) override {}

 private:
  SerialDataManager* serial_data_manager_;
  webrtc::scoped_refptr<webrtc::DataChannelInterface> data_channel_;
};

#endif
