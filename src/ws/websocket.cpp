#include "websocket.h"

#include <utility>

// Boost
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/post.hpp>
#include <boost/beast/core/buffers_to_string.hpp>
#include <boost/beast/websocket/stream.hpp>

#include "util.h"

Websocket::Websocket(boost::asio::io_context& ioc)
    : ws_(new websocket_t(ioc)), strand_(ws_->get_executor()) {
  ws_->write_buffer_bytes(8192);
}
Websocket::Websocket(boost::asio::io_context& ioc,
                     boost::asio::ssl::context ssl_ctx)
    : wss_(new ssl_websocket_t(ioc, ssl_ctx)), strand_(wss_->get_executor()) {
  wss_->write_buffer_bytes(8192);
}
Websocket::Websocket(boost::asio::ip::tcp::socket socket)
    : ws_(new websocket_t(std::move(socket))), strand_(ws_->get_executor()) {
  ws_->write_buffer_bytes(8192);
}

Websocket::~Websocket() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
}

bool Websocket::isSSL() const {
  return wss_ != nullptr;
}
Websocket::websocket_t& Websocket::NativeSocket() {
  return *ws_;
}
Websocket::ssl_websocket_t& Websocket::NativeSecureSocket() {
  return *wss_;
}

boost::asio::strand<Websocket::websocket_t::executor_type>&
Websocket::strand() {
  return strand_;
}

void Websocket::StartToRead(read_callback_t on_read) {
  boost::asio::post(strand_, std::bind(&Websocket::DoRead, this, on_read));
}

void Websocket::DoRead(read_callback_t on_read) {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  if (isSSL()) {
    wss_->async_read(
        read_buffer_,
        boost::asio::bind_executor(
            strand_, std::bind(&Websocket::OnRead, this, on_read,
                               std::placeholders::_1, std::placeholders::_2)));
  } else {
    ws_->async_read(
        read_buffer_,
        boost::asio::bind_executor(
            strand_, std::bind(&Websocket::OnRead, this, on_read,
                               std::placeholders::_1, std::placeholders::_2)));
  }
}

void Websocket::OnRead(read_callback_t on_read,
                       boost::system::error_code ec,
                       std::size_t bytes_transferred) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec.message();

  // エラーだろうが何だろうが on_read コールバック関数は必ず呼ぶ

  const auto text = boost::beast::buffers_to_string(read_buffer_.data());
  read_buffer_.consume(read_buffer_.size());

  on_read(ec, bytes_transferred, std::move(text));

  if (ec == boost::asio::error::operation_aborted)
    return;

  if (ec)
    return MOMO_BOOST_ERROR(ec, "OnRead");

  DoRead(on_read);
}

void Websocket::SendText(std::string text) {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  boost::asio::post(strand_,
                    std::bind(&Websocket::DoSendText, this, std::move(text)));
}

void Websocket::DoSendText(std::string text) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << text;

  bool empty = write_buffer_.empty();
  boost::beast::flat_buffer buffer;

  const auto n = boost::asio::buffer_copy(buffer.prepare(text.size()),
                                          boost::asio::buffer(text));
  RTC_LOG(LS_VERBOSE) << __FUNCTION__ << ": n=" << n;
  buffer.commit(n);

  {
    std::string tmp = boost::beast::buffers_to_string(buffer.data());
    RTC_LOG(LS_VERBOSE) << __FUNCTION__ << ": size=" << tmp.size()
                        << " text=" << tmp;
  }

  write_buffer_.push_back(std::move(buffer));

  if (empty) {
    DoWrite();
  }
}
void Websocket::DoWrite() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  auto& buffer = write_buffer_.front();

  RTC_LOG(LS_VERBOSE) << __FUNCTION__ << ": "
                      << boost::beast::buffers_to_string(buffer.data());

  {
    std::string tmp = boost::beast::buffers_to_string(buffer.data());
    RTC_LOG(LS_VERBOSE) << __FUNCTION__ << ": size=" << tmp.size()
                        << " text=" << tmp;
  }

  if (isSSL()) {
    wss_->text(true);
    wss_->async_write(
        buffer.data(),
        boost::asio::bind_executor(
            strand_, std::bind(&Websocket::OnWrite, this, std::placeholders::_1,
                               std::placeholders::_2)));
  } else {
    ws_->text(true);
    ws_->async_write(
        buffer.data(),
        boost::asio::bind_executor(
            strand_, std::bind(&Websocket::OnWrite, this, std::placeholders::_1,
                               std::placeholders::_2)));
  }
}

void Websocket::OnWrite(boost::system::error_code ec,
                        std::size_t bytes_transferred) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec.message();

  // エラーだろうが何だろうが on_write_ コールバック関数は必ず呼ぶ
  // on_write(ec, bytes_transferred);

  if (ec == boost::asio::error::operation_aborted)
    return;

  if (ec)
    return MOMO_BOOST_ERROR(ec, "OnWrite");

  {
    std::string tmp =
        boost::beast::buffers_to_string(write_buffer_.front().data());
    RTC_LOG(LS_VERBOSE) << __FUNCTION__
                        << ": bytes_transferred=" << bytes_transferred
                        << " size=" << tmp.size() << " text=" << tmp;
  }

  write_buffer_.erase(write_buffer_.begin());

  if (!write_buffer_.empty()) {
    DoWrite();
  }
}
