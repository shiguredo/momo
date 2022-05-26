#include "websocket.h"

#include <utility>

// WebRTC
#include <rtc_base/third_party/base64/base64.h>

// Boost
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/post.hpp>
#include <boost/beast/core/buffers_to_string.hpp>
#include <boost/beast/websocket/stream.hpp>

#include "ssl_verifier.h"
#include "util.h"

static std::shared_ptr<boost::asio::ssl::context> CreateSSLContext(
    const std::string& client_cert,
    const std::string& client_key) {
  // TLS 1.2 と 1.3 のみ対応
  SSL_CTX* handle = ::SSL_CTX_new(::TLS_method());
  SSL_CTX_set_min_proto_version(handle, TLS1_2_VERSION);
  SSL_CTX_set_max_proto_version(handle, TLS1_3_VERSION);
  auto ctx = std::make_shared<boost::asio::ssl::context>(handle);
  //ctx.set_default_verify_paths();
  ctx->set_options(boost::asio::ssl::context::default_workarounds |
                   boost::asio::ssl::context::no_sslv2 |
                   boost::asio::ssl::context::no_sslv3 |
                   boost::asio::ssl::context::single_dh_use);
  if (!client_cert.empty()) {
    ctx->use_certificate_file(client_cert,
                              boost::asio::ssl::context_base::file_format::pem);
    RTC_LOG(LS_INFO) << "client_cert=" << client_cert;
  }
  if (!client_key.empty()) {
    ctx->use_private_key_file(client_key,
                              boost::asio::ssl::context_base::file_format::pem);
    RTC_LOG(LS_INFO) << "client_key=" << client_key;
  }
  return ctx;
}

Websocket::Websocket(boost::asio::io_context& ioc)
    : ws_(new websocket_t(ioc)),
      resolver_(new boost::asio::ip::tcp::resolver(ioc)),
      strand_(ws_->get_executor()) {
  ws_->write_buffer_bytes(8192);
}
Websocket::Websocket(Websocket::ssl_tag,
                     boost::asio::io_context& ioc,
                     bool insecure,
                     const std::string& client_cert,
                     const std::string& client_key)
    : resolver_(new boost::asio::ip::tcp::resolver(ioc)),
      strand_(ioc.get_executor()),
      insecure_(insecure) {
  ssl_ctx_ = CreateSSLContext(client_cert, client_key);
  wss_.reset(new ssl_websocket_t(ioc, *ssl_ctx_));
  InitWss(wss_.get(), insecure);
}
Websocket::Websocket(boost::asio::ip::tcp::socket socket)
    : ws_(new websocket_t(std::move(socket))), strand_(ws_->get_executor()) {
  ws_->write_buffer_bytes(8192);
}
Websocket::Websocket(https_proxy_tag,
                     boost::asio::io_context& ioc,
                     bool insecure,
                     const std::string& client_cert,
                     const std::string& client_key,
                     std::string proxy_url,
                     std::string proxy_username,
                     std::string proxy_password)
    : resolver_(new boost::asio::ip::tcp::resolver(ioc)),
      strand_(ioc.get_executor()),
      https_proxy_(true),
      proxy_socket_(new boost::asio::ip::tcp::socket(ioc)),
      proxy_url_(std::move(proxy_url)),
      proxy_username_(std::move(proxy_username)),
      proxy_password_(std::move(proxy_password)) {
  ssl_ctx_ = CreateSSLContext(client_cert, client_key);
}

Websocket::~Websocket() {
  RTC_LOG(LS_INFO) << "Websocket::~Websocket this=" << (void*)this;
}

bool Websocket::IsSSL() const {
  return https_proxy_ || wss_ != nullptr;
}

void Websocket::InitWss(ssl_websocket_t* wss, bool insecure) {
  wss->write_buffer_bytes(8192);

  wss->next_layer().set_verify_mode(boost::asio::ssl::verify_peer);
  wss->next_layer().set_verify_callback(
      [insecure](bool preverified, boost::asio::ssl::verify_context& ctx) {
        if (preverified) {
          return true;
        }
        // insecure の場合は証明書をチェックしない
        if (insecure) {
          return true;
        }
        X509* cert = X509_STORE_CTX_get0_cert(ctx.native_handle());
        STACK_OF(X509)* chain = X509_STORE_CTX_get0_chain(ctx.native_handle());
        return SSLVerifier::VerifyX509(cert, chain);
      });
}

Websocket::websocket_t& Websocket::NativeSocket() {
  return *ws_;
}
Websocket::ssl_websocket_t& Websocket::NativeSecureSocket() {
  return *wss_;
}

void Websocket::Connect(const std::string& url, connect_callback_t on_connect) {
  // proxy 時は wss のみサポート
  if (https_proxy_) {
    ConnectProxy(url, std::move(on_connect));
    return;
  }

  if (!URLParts::Parse(url, parts_)) {
    on_connect(boost::system::errc::make_error_code(
        boost::system::errc::invalid_argument));
    return;
  }

  // wss と ws のみサポート
  if (parts_.scheme != "wss" && parts_.scheme != "ws") {
    on_connect(boost::system::errc::make_error_code(
        boost::system::errc::invalid_argument));
    return;
  }

  // コンストラクタの設定と接続のスキームが合っているか
  if (IsSSL() && parts_.scheme != "wss" || !IsSSL() && parts_.scheme != "ws") {
    on_connect(boost::system::errc::make_error_code(
        boost::system::errc::invalid_argument));
    return;
  }

  if (IsSSL()) {
    // SNI の設定を行う
    if (!SSL_set_tlsext_host_name(wss_->next_layer().native_handle(),
                                  parts_.host.c_str())) {
      boost::system::error_code ec{static_cast<int>(::ERR_get_error()),
                                   boost::asio::error::get_ssl_category()};
      on_connect(ec);
      return;
    }
  }

  on_connect_ = std::move(on_connect);

  // DNS ルックアップ
  resolver_->async_resolve(
      parts_.host, parts_.GetPort(),
      std::bind(&Websocket::OnResolve, this, std::placeholders::_1,
                std::placeholders::_2));
}

void Websocket::OnResolve(
    boost::system::error_code ec,
    boost::asio::ip::tcp::resolver::results_type results) {
  if (ec) {
    auto on_connect = std::move(on_connect_);
    on_connect(ec);
    return;
  }

  // DNS ルックアップで得られたエンドポイントに対して接続する
  if (IsSSL()) {
    boost::asio::async_connect(
        wss_->next_layer().next_layer(), results.begin(), results.end(),
        std::bind(&Websocket::OnSSLConnect, this, std::placeholders::_1));
  } else {
    boost::asio::async_connect(
        ws_->next_layer(), results.begin(), results.end(),
        std::bind(&Websocket::OnConnect, this, std::placeholders::_1));
  }
}

void Websocket::OnSSLConnect(boost::system::error_code ec) {
  if (ec) {
    auto on_connect = std::move(on_connect_);
    on_connect(ec);
    return;
  }

  // SSL のハンドシェイク
  wss_->next_layer().async_handshake(
      boost::asio::ssl::stream_base::client,
      std::bind(&Websocket::OnSSLHandshake, this, std::placeholders::_1));
}

void Websocket::OnSSLHandshake(boost::system::error_code ec) {
  if (ec) {
    RTC_LOG(LS_ERROR) << "Failed to OnSSLHandshake";
    auto on_connect = std::move(on_connect_);
    on_connect(ec);
    return;
  }

  // Websocket のハンドシェイク
  wss_->async_handshake(
      parts_.host, parts_.path_query_fragment,
      std::bind(&Websocket::OnHandshake, this, std::placeholders::_1));
}

void Websocket::OnConnect(boost::system::error_code ec) {
  if (ec) {
    auto on_connect = std::move(on_connect_);
    on_connect(ec);
    return;
  }

  // Websocket のハンドシェイク
  ws_->async_handshake(
      parts_.host, parts_.path_query_fragment,
      std::bind(&Websocket::OnHandshake, this, std::placeholders::_1));
}

void Websocket::OnHandshake(boost::system::error_code ec) {
  auto on_connect = std::move(on_connect_);
  on_connect(ec);
}

void Websocket::Accept(
    boost::beast::http::request<boost::beast::http::string_body> req,
    connect_callback_t on_connect) {
  on_connect_ = std::move(on_connect);
  ws_->async_accept(
      req, std::bind(&Websocket::OnAccept, this, std::placeholders::_1));
}

void Websocket::OnAccept(boost::system::error_code ec) {
  auto on_connect = std::move(on_connect_);
  on_connect(ec);
}

void Websocket::Read(read_callback_t on_read) {
  boost::asio::post(strand_,
                    std::bind(&Websocket::DoRead, this, std::move(on_read)));
}

void Websocket::DoRead(read_callback_t on_read) {
  if (IsSSL()) {
    wss_->async_read(read_buffer_,
                     std::bind(&Websocket::OnRead, this, std::move(on_read),
                               std::placeholders::_1, std::placeholders::_2));
  } else {
    ws_->async_read(read_buffer_,
                    std::bind(&Websocket::OnRead, this, std::move(on_read),
                              std::placeholders::_1, std::placeholders::_2));
  }
}

void Websocket::ConnectProxy(const std::string& url,
                             connect_callback_t on_connect) {
  if (!URLParts::Parse(url, parts_)) {
    on_connect(boost::system::errc::make_error_code(
        boost::system::errc::invalid_argument));
    return;
  }

  if (parts_.scheme != "wss") {
    on_connect(boost::system::errc::make_error_code(
        boost::system::errc::invalid_argument));
    return;
  }

  if (!URLParts::Parse(proxy_url_, proxy_parts_)) {
    on_connect(boost::system::errc::make_error_code(
        boost::system::errc::invalid_argument));
    return;
  }

  if (!URLParts::Parse(url, parts_)) {
    on_connect(boost::system::errc::make_error_code(
        boost::system::errc::invalid_argument));
    return;
  }

  // 今は http + wss の組み合わせしか許可しない
  if (proxy_parts_.scheme != "http" || parts_.scheme != "wss") {
    on_connect(boost::system::errc::make_error_code(
        boost::system::errc::invalid_argument));
    return;
  }

  on_connect_ = std::move(on_connect);

  // proxy サーバーの DNS 解決を行う
  resolver_->async_resolve(
      proxy_parts_.host, proxy_parts_.GetPort(),
      std::bind(&Websocket::OnResolveProxy, this, std::placeholders::_1,
                std::placeholders::_2));
}

void Websocket::OnResolveProxy(
    boost::system::error_code ec,
    boost::asio::ip::tcp::resolver::results_type results) {
  if (ec) {
    auto on_connect = std::move(on_connect_);
    on_connect(ec);
    return;
  }

  boost::asio::async_connect(
      *proxy_socket_, results.begin(), results.end(),
      std::bind(&Websocket::OnConnectProxy, this, std::placeholders::_1));
}

void Websocket::OnConnectProxy(boost::system::error_code ec) {
  if (ec) {
    auto on_connect = std::move(on_connect_);
    on_connect(ec);
    return;
  }

  // CONNECT 送信
  auto target = parts_.host + ":" + parts_.GetPort();
  proxy_req_.method(boost::beast::http::verb::connect);
  proxy_req_.target(target);
  proxy_req_.version(11);
  proxy_req_.set(boost::beast::http::field::host, target);
  proxy_req_.set(
      boost::beast::http::field::proxy_authorization,
      "Basic " + rtc::Base64::Encode(proxy_username_ + ":" + proxy_password_));
  boost::beast::http::async_write(
      *proxy_socket_, proxy_req_,
      std::bind(&Websocket::OnWriteProxy, this, std::placeholders::_1,
                std::placeholders::_2));
}

void Websocket::OnWriteProxy(boost::system::error_code ec,
                             std::size_t bytes_transferred) {
  if (ec) {
    auto on_connect = std::move(on_connect_);
    on_connect(ec);
    return;
  }

  // CONNECT のレスポンス読み込み
  proxy_resp_parser_.reset(
      new boost::beast::http::response_parser<boost::beast::http::empty_body>(
          proxy_resp_));
  // see https://stackoverflow.com/a/49837467/10904212
  proxy_resp_parser_->skip(true);
  boost::beast::http::async_read(
      *proxy_socket_, proxy_buffer_, *proxy_resp_parser_,
      std::bind(&Websocket::OnReadProxy, this, std::placeholders::_1,
                std::placeholders::_2));
}

void Websocket::OnReadProxy(boost::system::error_code ec,
                            std::size_t bytes_transferred) {
  if (ec) {
    auto on_connect = std::move(on_connect_);
    on_connect(ec);
    return;
  }

  // wss を作って、あとは普通の SSL ハンドシェイクを行う
  wss_.reset(new ssl_websocket_t(std::move(*proxy_socket_), *ssl_ctx_));
  InitWss(wss_.get(), insecure_);

  // SNI の設定を行う
  if (!SSL_set_tlsext_host_name(wss_->next_layer().native_handle(),
                                parts_.host.c_str())) {
    boost::system::error_code ec{static_cast<int>(::ERR_get_error()),
                                 boost::asio::error::get_ssl_category()};
    auto on_connect = std::move(on_connect_);
    on_connect(ec);
    return;
  }

  wss_->next_layer().async_handshake(
      boost::asio::ssl::stream_base::client,
      std::bind(&Websocket::OnSSLHandshake, this, std::placeholders::_1));
}

void Websocket::OnRead(read_callback_t on_read,
                       boost::system::error_code ec,
                       std::size_t bytes_transferred) {
  RTC_LOG(LS_INFO) << "Websocket::OnRead this=" << (void*)this
                   << " ec=" << ec.message();

  if (ec) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": " << ec.message();
  }

  std::string text;
  if (!ec) {
    text = boost::beast::buffers_to_string(read_buffer_.data());
    read_buffer_.consume(read_buffer_.size());
  }

  std::move(on_read)(ec, bytes_transferred, std::move(text));
}

void Websocket::WriteText(std::string text, write_callback_t on_write) {
  boost::asio::post(strand_, std::bind(&Websocket::DoWriteText, this,
                                       std::move(text), std::move(on_write)));
}

void Websocket::DoWriteText(std::string text, write_callback_t on_write) {
  bool empty = write_data_.empty();
  boost::beast::flat_buffer buffer;

  const auto n = boost::asio::buffer_copy(buffer.prepare(text.size()),
                                          boost::asio::buffer(text));
  buffer.commit(n);

  write_data_.emplace_back(
      new WriteData{std::move(buffer), std::move(on_write), true});

  if (empty) {
    DoWrite();
  }
}
void Websocket::DoWrite() {
  auto& data = write_data_.front();

  RTC_LOG(LS_VERBOSE) << __FUNCTION__ << ": "
                      << boost::beast::buffers_to_string(data->buffer.data());

  if (IsSSL()) {
    wss_->text(data->text);
    wss_->async_write(data->buffer.data(),
                      std::bind(&Websocket::OnWrite, this,
                                std::placeholders::_1, std::placeholders::_2));
  } else {
    ws_->text(data->text);
    ws_->async_write(data->buffer.data(),
                     std::bind(&Websocket::OnWrite, this, std::placeholders::_1,
                               std::placeholders::_2));
  }
}

void Websocket::OnWrite(boost::system::error_code ec,
                        std::size_t bytes_transferred) {
  RTC_LOG(LS_INFO) << "Websocket::OnWrite this=" << (void*)this
                   << " ec=" << ec.message();

  if (ec) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": " << ec.message();
  }

  auto& data = write_data_.front();
  if (data->callback) {
    std::move(data->callback)(ec, bytes_transferred);
  }

  write_data_.erase(write_data_.begin());

  if (!write_data_.empty()) {
    DoWrite();
  }
}

void Websocket::Close(close_callback_t on_close) {
  boost::asio::post(strand_,
                    std::bind(&Websocket::DoClose, this, std::move(on_close)));
}

void Websocket::DoClose(close_callback_t on_close) {
  if (IsSSL()) {
    wss_->async_close(boost::beast::websocket::close_code::normal,
                      std::bind(&Websocket::OnClose, this, std::move(on_close),
                                std::placeholders::_1));
  } else {
    ws_->async_close(boost::beast::websocket::close_code::normal,
                     std::bind(&Websocket::OnClose, this, std::move(on_close),
                               std::placeholders::_1));
  }
}

void Websocket::OnClose(close_callback_t on_close,
                        boost::system::error_code ec) {
  RTC_LOG(LS_INFO) << "Websocket::OnClose this=" << (void*)this
                   << " ec=" << ec.message();

  on_close(ec);
}
