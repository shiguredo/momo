#ifndef SERIAL_DATA_MANAGER_H_
#define SERIAL_DATA_MANAGER_H_
#include <vector>

#include <boost/asio.hpp>

#include "rtc/data_manager.h"
#include "rtc_base/critical_section.h"
#include "serial_data_channel.h"

class SerialDataChannel;

class SerialDataManager : public RTCDataManager {
 public:
  SerialDataManager(std::string device, unsigned int rate);
  ~SerialDataManager();

  void OnDataChannel(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) override;
  void OnClosed(SerialDataChannel* serial_data_channel);

  void send(const uint8_t* data, size_t length);

 private:
  void doCloseSerial();
  void doRead();
  void onRead(const boost::system::error_code& error, size_t bytes_transferred);
  void startWrite();
  void doWrite();
  void onWrite(const boost::system::error_code& error);

  std::vector<SerialDataChannel*> _serial_data_channels;
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;

  std::unique_ptr<uint8_t[]> read_buffer_;
  size_t read_buffer_size_;
  std::vector<uint8_t> read_line_buffer_;

  rtc::CriticalSection write_buffer_lock_;
  bool writting_;
  std::vector<uint8_t> write_buffer_;
  size_t write_length_;
  std::thread read_thread_;
};

#endif