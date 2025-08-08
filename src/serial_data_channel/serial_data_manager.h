#ifndef SERIAL_DATA_MANAGER_H_
#define SERIAL_DATA_MANAGER_H_

#include <memory>
#include <vector>

// Boost
#include <boost/asio.hpp>

// WebRTC
#include <rtc_base/synchronization/mutex.h>

#include "rtc/rtc_data_manager.h"
#include "serial_data_channel.h"

class SerialDataChannel;

class SerialDataManager : public RTCDataManager {
 public:
  static std::unique_ptr<SerialDataManager> Create(boost::asio::io_context& ioc,
                                                   std::string device,
                                                   unsigned int rate) {
    std::unique_ptr<SerialDataManager> data_manager(new SerialDataManager(ioc));
    if (!data_manager->Connect(device, rate)) {
      return nullptr;
    }
    return data_manager;
  }
  ~SerialDataManager();

  void Send(const uint8_t* data, size_t length);

  void OnDataChannel(webrtc::scoped_refptr<webrtc::DataChannelInterface>
                         data_channel) override;
  void OnClosed(SerialDataChannel* serial_data_channel);

 private:
  SerialDataManager(boost::asio::io_context& ioc);
  bool Connect(std::string device, unsigned int rate);
  void DoCloseSerial();
  void DoRead();
  void OnRead(const boost::system::error_code& error, size_t bytes_transferred);
  void SendLineFromSerial();
  void StartWrite(std::vector<uint8_t> v);
  void DoWrite();
  void OnWrite(const boost::system::error_code& error);

  boost::asio::serial_port serial_port_;
  std::function<void(std::function<void()>)> post_;
  webrtc::Mutex channels_lock_;
  std::vector<SerialDataChannel*> serial_data_channels_;

  std::unique_ptr<uint8_t[]> read_buffer_;
  size_t read_buffer_size_;
  std::vector<uint8_t> read_line_buffer_;

  std::vector<uint8_t> write_buffer_;
  size_t write_length_;
};

#endif
