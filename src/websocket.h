#ifndef WEBSOCKET_H_
#define WEBSOCKET_H_

#include <functional>
#include <memory>

// Boost
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl/context.hpp>
#include <boost/asio/ssl/stream.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/core/multi_buffer.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <boost/beast/websocket/stream.hpp>

#include "url_parts.h"

// SSL+クライアント、非SSL+クライアント、サーバで大体同じように扱える WebSocket。
//
// 任意のスレッドから WriteText を呼ぶことで書き込みができ、
// 書き込み完了のコールバックを待たずに次の WriteText を呼ぶことができる。
class Websocket {
 public:
  typedef boost::beast::websocket::stream<boost::asio::ip::tcp::socket>
      websocket_t;
  typedef boost::beast::websocket::stream<
      boost::asio::ssl::stream<boost::asio::ip::tcp::socket>>
      ssl_websocket_t;
  typedef std::function<void(boost::system::error_code ec)> connect_callback_t;
  typedef std::function<void(boost::system::error_code ec,
                             std::size_t bytes_transferred,
                             std::string text)>
      read_callback_t;
  typedef std::function<void(boost::system::error_code ec,
                             std::size_t bytes_transferred)>
      write_callback_t;
  typedef std::function<void(boost::system::error_code ec)> close_callback_t;

  struct ssl_tag {};

 public:
  // 非SSL+クライアント
  Websocket(boost::asio::io_context& ioc);
  // SSL+クライアント
  Websocket(ssl_tag,
            boost::asio::io_context& ioc,
            bool insecure,
            const std::string& client_cert,
            const std::string& client_key);

 private:
  // SSL+クライアントの内部用コンストラクタ
  Websocket(ssl_tag,
            boost::asio::io_context& ioc,
            bool insecure,
            boost::asio::ssl::context ssl_ctx);

 public:
  // サーバ
  Websocket(boost::asio::ip::tcp::socket socket);
  ~Websocket();

  // WebSocket クライアントの接続確立
  void Connect(const std::string& url, connect_callback_t on_connect);

  // WebSocket サーバの接続確立
  void Accept(boost::beast::http::request<boost::beast::http::string_body> req,
              connect_callback_t on_connect);

  void Read(read_callback_t on_read);
  void WriteText(std::string text, write_callback_t on_write = nullptr);
  void Close(close_callback_t on_close);

  websocket_t& NativeSocket();
  ssl_websocket_t& NativeSecureSocket();

 private:
  bool IsSSL() const;

  void OnResolve(boost::system::error_code ec,
                 boost::asio::ip::tcp::resolver::results_type results);
  void OnSSLConnect(boost::system::error_code ec);
  void OnSSLHandshake(boost::system::error_code ec);
  void OnConnect(boost::system::error_code ec);
  void OnHandshake(boost::system::error_code ec);
  void OnAccept(boost::system::error_code ec);

  void DoRead(read_callback_t on_read);
  void OnRead(read_callback_t on_read,
              boost::system::error_code ec,
              std::size_t bytes_transferred);

  void DoClose(close_callback_t on_close);
  void OnClose(close_callback_t on_close, boost::system::error_code ec);

 private:
  void DoWriteText(std::string text, write_callback_t on_write);
  void DoWrite();
  void OnWrite(boost::system::error_code ec, std::size_t bytes_transferred);

 private:
  std::unique_ptr<websocket_t> ws_;
  std::unique_ptr<ssl_websocket_t> wss_;

  std::unique_ptr<boost::asio::ip::tcp::resolver> resolver_;
  connect_callback_t on_connect_;
  URLParts parts_;

  boost::asio::strand<websocket_t::executor_type> strand_;

  boost::beast::multi_buffer read_buffer_;
  struct WriteData {
    boost::beast::flat_buffer buffer;
    write_callback_t callback;
    bool text;
  };
  std::vector<std::unique_ptr<WriteData>> write_data_;
};

#endif  // WEBSOCKET_H_
