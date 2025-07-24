// こちらを参考にさせていただきました
// https://github.com/fedetft/serial-port/blob/master/4_callback/AsyncSerial.cpp

#include "serial_data_manager.h"

#include <functional>
#include <iostream>

// Boost
#include <boost/asio/post.hpp>

// WebRTC
#include <rtc_base/log_sinks.h>

// Boost
#include <boost/asio/post.hpp>

#define SERIAL_TX_BUFFER_SIZE 16
#define SERIAL_RX_BUFFER_SIZE 256

SerialDataManager::SerialDataManager(boost::asio::io_context& ioc)
    : serial_port_(ioc), read_buffer_size_(SERIAL_RX_BUFFER_SIZE) {
  post_ = [&ioc](std::function<void()> f) {
    if (ioc.stopped())
      return;
    boost::asio::post(ioc, f);
  };
}

SerialDataManager::~SerialDataManager() {
  {
    webrtc::MutexLock lock(&channels_lock_);
    for (SerialDataChannel* serial_data_channel : serial_data_channels_) {
      delete serial_data_channel;
    }
  }

  DoCloseSerial();
}

void SerialDataManager::OnDataChannel(
    webrtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
  webrtc::MutexLock lock(&channels_lock_);
  serial_data_channels_.push_back(new SerialDataChannel(this, data_channel));
}

void SerialDataManager::OnClosed(SerialDataChannel* serial_data_channel) {
  webrtc::MutexLock lock(&channels_lock_);
  serial_data_channels_.erase(
      std::remove(serial_data_channels_.begin(), serial_data_channels_.end(),
                  serial_data_channel),
      serial_data_channels_.end());
  delete serial_data_channel;
}

bool SerialDataManager::Connect(std::string device, unsigned int rate) {
  boost::system::error_code error;
  serial_port_.open(device, error);
  if (error) {
    std::cerr << "failed to connect serial port device : " << device
              << std::endl;
    return false;
  }

  serial_port_.set_option(boost::asio::serial_port_base::baud_rate(rate),
                          error);
  if (error) {
    std::cerr << "failed to set serial port baudrate : " << rate << std::endl;
    return false;
  }
  serial_port_.set_option(boost::asio::serial_port_base::character_size(8),
                          error);
  if (error) {
    std::cerr << "failed to set serial port character size : 8" << std::endl;
    return false;
  }
  serial_port_.set_option(
      boost::asio::serial_port_base::flow_control(
          boost::asio::serial_port_base::flow_control::none),
      error);
  if (error) {
    std::cerr << "failed to set serial port flow control : none" << std::endl;
    return false;
  }
  serial_port_.set_option(boost::asio::serial_port_base::parity(
                              boost::asio::serial_port_base::parity::none),
                          error);
  if (error) {
    std::cerr << "failed to set serial port parity : none" << std::endl;
    return false;
  }
  serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
                              boost::asio::serial_port_base::stop_bits::one),
                          error);
  if (error) {
    std::cerr << "failed to set serial port stop bit : one" << std::endl;
    return false;
  }

  read_buffer_.reset(new uint8_t[read_buffer_size_]);
  post_(std::bind(&SerialDataManager::DoRead, this));
  return true;
}

void SerialDataManager::Send(const uint8_t* data, size_t length) {
  std::vector<uint8_t> v(data, data + length);
  post_(std::bind(&SerialDataManager::StartWrite, this, std::move(v)));
}

void SerialDataManager::DoCloseSerial() {
  if (!serial_port_.is_open()) {
    return;
  }
  boost::system::error_code error;
  serial_port_.cancel(error);
  serial_port_.close(error);
}

void SerialDataManager::DoRead() {
  if (!serial_port_.is_open()) {
    return;
  }
  serial_port_.async_read_some(
      boost::asio::buffer(read_buffer_.get(), read_buffer_size_),
      std::bind(&SerialDataManager::OnRead, this, std::placeholders::_1,
                std::placeholders::_2));
}

void SerialDataManager::OnRead(const boost::system::error_code& error,
                               size_t bytes_transferred) {
  if (error) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " async_read_some failed  error :" << error;
    DoCloseSerial();
    return;
  }
  read_line_buffer_.insert(read_line_buffer_.end(), read_buffer_.get(),
                           read_buffer_.get() + bytes_transferred);
  {
    webrtc::MutexLock lock(&channels_lock_);
    SendLineFromSerial();
  }
  DoRead();
}

void SerialDataManager::SendLineFromSerial() {
  auto delimiter_iterator =
      std::find(read_line_buffer_.begin(), read_line_buffer_.end(), '\n');
  if (delimiter_iterator != read_line_buffer_.end()) {
    size_t delimiter_index =
        std::distance(read_line_buffer_.begin(), delimiter_iterator);
    for (SerialDataChannel* serial_data_channel : serial_data_channels_) {
      serial_data_channel->Send(read_line_buffer_.data(), delimiter_index);
    }
    read_line_buffer_.erase(read_line_buffer_.begin(), delimiter_iterator + 1);
    SendLineFromSerial();
  }
}

void SerialDataManager::StartWrite(std::vector<uint8_t> v) {
  if (!serial_port_.is_open()) {
    return;
  }
  bool empty = write_buffer_.empty();
  write_buffer_.insert(write_buffer_.end(), v.begin(), v.end());
  if (empty && !write_buffer_.empty()) {
    DoWrite();
  }
}

void SerialDataManager::DoWrite() {
  if (write_buffer_.size() < SERIAL_TX_BUFFER_SIZE) {
    write_length_ = write_buffer_.size();
  } else {
    write_length_ = SERIAL_TX_BUFFER_SIZE;
  }
  async_write(
      serial_port_, boost::asio::buffer(write_buffer_.data(), write_length_),
      std::bind(&SerialDataManager::OnWrite, this, std::placeholders::_1));
}

void SerialDataManager::OnWrite(const boost::system::error_code& error) {
  if (error) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " async_write failed  error :" << error;
    DoCloseSerial();
    return;
  }
  write_buffer_.erase(write_buffer_.begin(),
                      write_buffer_.begin() + write_length_);
  if (write_buffer_.empty()) {
    return;
  }
  DoWrite();
}
