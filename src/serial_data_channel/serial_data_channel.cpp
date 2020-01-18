#include "serial_data_channel.h"

#include "rtc_base/logging.h"

SerialDataChannel::SerialDataChannel(
    SerialDataManager* serial_data_manager,
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel)
    : serial_data_manager_(serial_data_manager), data_channel_(data_channel) {
  data_channel_->RegisterObserver(this);
}

void SerialDataChannel::OnStateChange() {
  webrtc::DataChannelInterface::DataState state = data_channel_->state();
  if (state == webrtc::DataChannelInterface::kClosed) {
    serial_data_manager_->OnClosed(this);
  }
}

void SerialDataChannel::OnMessage(const webrtc::DataBuffer& buffer) {
  const uint8_t* data = buffer.data.data<uint8_t>();
  size_t lenght = buffer.data.size();

  serial_data_manager_->send(data, lenght);
}

void SerialDataChannel::send(uint8_t* data, size_t length) {
  if (data_channel_->state() != webrtc::DataChannelInterface::kOpen) {
    return;
  }
  rtc::CopyOnWriteBuffer buffer(data, length);
  webrtc::DataBuffer data_buffer(buffer, true);
  data_channel_->Send(data_buffer);
}