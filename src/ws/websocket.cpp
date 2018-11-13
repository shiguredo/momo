#include "websocket.h"
#include "util.h"

#include <utility>

Websocket::Websocket(boost::asio::io_context& ioc, read_callback_t on_read, write_callback_t on_write)
    : ws_(new websocket_t(ioc))
    , on_read_(on_read)
    , on_write_(on_write)
    , strand_(ws_->get_executor()) { }
Websocket::Websocket(boost::asio::io_context& ioc, boost::asio::ssl::context ssl_ctx, read_callback_t on_read, write_callback_t on_write)
    : wss_(new ssl_websocket_t(ioc, ssl_ctx))
    , on_read_(on_read)
    , on_write_(on_write)
    , strand_(wss_->get_executor()) { }
Websocket::Websocket(boost::asio::ip::tcp::socket socket, read_callback_t on_read, write_callback_t on_write)
    : ws_(new websocket_t(std::move(socket)))
    , on_read_(on_read)
    , on_write_(on_write)
    , strand_(ws_->get_executor()) { }

bool Websocket::isSSL() const { return wss_ != nullptr; }
Websocket::websocket_t& Websocket::nativeSocket() { return *ws_; }
Websocket::ssl_websocket_t& Websocket::nativeSecureSocket() { return *wss_; }

boost::asio::strand<boost::asio::io_context::executor_type>& Websocket::strand() { return strand_; }

void Websocket::startToRead() {
    boost::asio::post(
        strand_,
        std::bind(
            &Websocket::doRead,
            shared_from_this()));
}

void Websocket::doRead() {
    RTC_LOG(LS_INFO) << __FUNCTION__;

    if (isSSL()) {
        wss_->async_read(
            read_buffer_,
            boost::asio::bind_executor(
                strand_,
                std::bind(
                    &Websocket::onRead,
                    shared_from_this(),
                    std::placeholders::_1,
                    std::placeholders::_2)));
    } else {
        ws_->async_read(
            read_buffer_,
            boost::asio::bind_executor(
                strand_,
                std::bind(
                    &Websocket::onRead,
                    shared_from_this(),
                    std::placeholders::_1,
                    std::placeholders::_2)));
    }
}

void Websocket::onRead(boost::system::error_code ec, std::size_t bytes_transferred) {
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec.message();

    // エラーだろうが何だろうが on_read_ コールバック関数は必ず呼ぶ

    const auto text = boost::beast::buffers_to_string(read_buffer_.data());
    read_buffer_.consume(read_buffer_.size());

    on_read_(ec, bytes_transferred, std::move(text));

    if (ec == boost::asio::error::operation_aborted)
        return;

    if (ec)
        return MOMO_BOOST_ERROR(ec, "onRead");

    doRead();
}

void Websocket::sendText(std::string text) {
    RTC_LOG(LS_INFO) << __FUNCTION__;
    boost::asio::post(
        strand_,
        std::bind(
            &Websocket::doSendText,
            shared_from_this(),
            std::move(text)));
}

void Websocket::doSendText(std::string text) {
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << text;

    bool empty = write_buffer_.size() == 0;

    const auto n = boost::asio::buffer_copy(write_buffer_.prepare(text.size()), boost::asio::buffer(text));
    write_buffer_.commit(n);

    if (empty) {
        doWrite();
    }
}
void Websocket::doWrite() {
    RTC_LOG(LS_INFO) << __FUNCTION__;

    if (isSSL()) {
        wss_->text(true);
        wss_->async_write(
            write_buffer_.data(),
            boost::asio::bind_executor(
                strand_,
                std::bind(
                    &Websocket::onWrite,
                    shared_from_this(),
                    std::placeholders::_1,
                    std::placeholders::_2)));
    } else {
        ws_->text(true);
        ws_->async_write(
            write_buffer_.data(),
            boost::asio::bind_executor(
                strand_,
                std::bind(
                    &Websocket::onWrite,
                    shared_from_this(),
                    std::placeholders::_1,
                    std::placeholders::_2)));
    }
}

void Websocket::onWrite(boost::system::error_code ec, std::size_t bytes_transferred)
{
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec.message();

    // エラーだろうが何だろうが on_write_ コールバック関数は必ず呼ぶ
    on_write_(ec, bytes_transferred);

    if (ec == boost::asio::error::operation_aborted)
        return;

    if (ec)
        return MOMO_BOOST_ERROR(ec, "onWrite");

    write_buffer_.consume(bytes_transferred);

    if (write_buffer_.size() != 0)
    {
        doWrite();
    }
}
