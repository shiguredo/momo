// こちらを参考にさせていただきました
// https://github.com/fedetft/serial-port/blob/master/4_callback/AsyncSerial.cpp

#include "serial_data_manager.h"

#include <boost/bind.hpp>

#include "rtc_base/log_sinks.h"

#define SERIAL_TX_BUFFER_SIZE 16
#define SERIAL_RX_BUFFER_SIZE 256

SerialDataManager::SerialDataManager(std::string device, unsigned int rate)
    : serial_port_(io_service_),
      read_buffer_size_(SERIAL_RX_BUFFER_SIZE),
      writting_(false) {
  boost::system::error_code error;
  serial_port_.open(device, error);
  if (error) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " open serial failed  device : " << device;
    return;
  }

  serial_port_.set_option(boost::asio::serial_port_base::baud_rate(rate),
                          error);
  serial_port_.set_option(boost::asio::serial_port_base::character_size(8),
                          error);
  serial_port_.set_option(
      boost::asio::serial_port_base::flow_control(
          boost::asio::serial_port_base::flow_control::none),
      error);
  serial_port_.set_option(boost::asio::serial_port_base::parity(
                              boost::asio::serial_port_base::parity::none),
                          error);
  serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
                              boost::asio::serial_port_base::stop_bits::one),
                          error);

  read_buffer_.reset(new uint8_t[read_buffer_size_]);
  io_service_.post(boost::bind(&SerialDataManager::doRead, this));
  read_thread_ =
      std::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}

SerialDataManager::~SerialDataManager() {
  for (SerialDataChannel* serial_data_channel : _serial_data_channels) {
    delete serial_data_channel;
  }

  io_service_.post(boost::bind(&SerialDataManager::doCloseSerial, this));
  read_thread_.join();
  io_service_.reset();
}

void SerialDataManager::OnDataChannel(
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
  _serial_data_channels.push_back(new SerialDataChannel(this, data_channel));
}

void SerialDataManager::OnClosed(SerialDataChannel* serial_data_channel) {
  _serial_data_channels.erase(
      std::remove(_serial_data_channels.begin(), _serial_data_channels.end(),
                  serial_data_channel),
      _serial_data_channels.end());
  delete serial_data_channel;
}

void SerialDataManager::send(const uint8_t* data, size_t length) {
  if (!serial_port_.is_open()) {
    return;
  }
  {
    rtc::CritScope lock(&write_buffer_lock_);
    write_buffer_.insert(write_buffer_.end(), data, data + length);
  }
  io_service_.post(boost::bind(&SerialDataManager::startWrite, this));
}

void SerialDataManager::doCloseSerial() {
  if (!serial_port_.is_open()) {
    return;
  }
  boost::system::error_code error;
  serial_port_.cancel(error);
  serial_port_.close(error);
}

void SerialDataManager::doRead() {
  if (!serial_port_.is_open()) {
    return;
  }
  serial_port_.async_read_some(
      boost::asio::buffer(read_buffer_.get(), read_buffer_size_),
      boost::bind(&SerialDataManager::onRead, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void SerialDataManager::onRead(const boost::system::error_code& error,
                               size_t bytes_transferred) {
  if (error) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " async_read_some failed  error :" << error;
    doCloseSerial();
    return;
  }
  read_line_buffer_.insert(read_line_buffer_.end(), read_buffer_.get(),
                           read_buffer_.get() + bytes_transferred);
  auto delimiter_iterator =
      std::find(read_line_buffer_.begin(), read_line_buffer_.end(), '\n');
  if (delimiter_iterator != read_line_buffer_.end()) {
    size_t delimiter_index =
        std::distance(read_line_buffer_.begin(), delimiter_iterator);
    for (SerialDataChannel* serial_data_channel : _serial_data_channels) {
      serial_data_channel->send(read_line_buffer_.data(), delimiter_index);
    }
    read_line_buffer_.erase(read_line_buffer_.begin(), delimiter_iterator + 1);
  }
  doRead();
}

void SerialDataManager::startWrite() {
  rtc::CritScope lock(&write_buffer_lock_);
  if (writting_ || write_buffer_.size() == 0) {
    return;
  }
  writting_ = true;
  doWrite();
}

void SerialDataManager::doWrite() {
  if (write_buffer_.size() < SERIAL_TX_BUFFER_SIZE) {
    write_length_ = write_buffer_.size();
  } else {
    write_length_ = SERIAL_TX_BUFFER_SIZE;
  }
  async_write(serial_port_,
              boost::asio::buffer(write_buffer_.data(), write_length_),
              boost::bind(&SerialDataManager::onWrite, this,
                          boost::asio::placeholders::error));
}

void SerialDataManager::onWrite(const boost::system::error_code& error) {
  if (error) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " async_write failed  error :" << error;
    doCloseSerial();
    return;
  }
  rtc::CritScope lock(&write_buffer_lock_);
  write_buffer_.erase(write_buffer_.begin(),
                      write_buffer_.begin() + write_length_);
  if (write_buffer_.size() == 0) {
    writting_ = false;
    return;
  }
  doWrite();
}