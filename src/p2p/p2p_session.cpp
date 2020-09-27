#include "p2p_session.h"

// Boost
#include <boost/beast/core/error.hpp>
#include <boost/beast/http/empty_body.hpp>
#include <boost/beast/http/error.hpp>
#include <boost/beast/http/file_body.hpp>
#include <boost/beast/http/read.hpp>
#include <boost/beast/version.hpp>
#include <boost/beast/websocket/rfc6455.hpp>
#include <boost/filesystem/path.hpp>

#ifdef _WIN32
#include <codecvt>
#endif

#include "util.h"

P2PSession::P2PSession(boost::asio::io_context& ioc,
                       boost::asio::ip::tcp::socket socket,
                       RTCManager* rtc_manager,
                       P2PSessionConfig config)
    : ioc_(ioc),
      socket_(std::move(socket)),
      strand_(socket_.get_executor()),
      rtc_manager_(rtc_manager),
      config_(std::move(config)) {}

// Start the asynchronous operation
void P2PSession::Run() {
  DoRead();
}

void P2PSession::DoRead() {
  // Make the request empty before reading,
  // otherwise the operation behavior is undefined.
  req_ = {};

  // Read a request
  boost::beast::http::async_read(
      socket_, buffer_, req_,
      boost::asio::bind_executor(
          strand_, std::bind(&P2PSession::OnRead, shared_from_this(),
                             std::placeholders::_1, std::placeholders::_2)));
}

void P2PSession::OnRead(boost::system::error_code ec,
                        std::size_t bytes_transferred) {
  boost::ignore_unused(bytes_transferred);

  // 接続が切られた
  if (ec == boost::beast::http::error::end_of_stream)
    return DoClose();

  if (ec)
    return MOMO_BOOST_ERROR(ec, "read");

  // WebSocket の upgrade リクエスト
  if (req_.target() == "/ws") {
    if (boost::beast::websocket::is_upgrade(req_)) {
      P2PWebsocketSessionConfig config;
      config.no_google_stun = config_.no_google_stun;
      P2PWebsocketSession::Create(ioc_, std::move(socket_), rtc_manager_,
                                  std::move(config))
          ->Run(std::move(req_));
      return;
    } else {
      SendResponse(Util::BadRequest(std::move(req_), "Not upgrade request"));
      return;
    }
  }

  HandleRequest();
}

void P2PSession::HandleRequest() {
  boost::beast::http::request<boost::beast::http::string_body> req(
      std::move(req_));

  // Make sure we can handle the method
  if (req.method() != boost::beast::http::verb::get &&
      req.method() != boost::beast::http::verb::head)
    return SendResponse(
        Util::BadRequest(std::move(req), "Unknown HTTP-method"));

  // Request path must be absolute and not contain "..".
  if (req.target().empty() || req.target()[0] != '/' ||
      req.target().find("..") != boost::beast::string_view::npos)
    return SendResponse(Util::BadRequest(req, "Illegal request-target"));

  // Build the path to the requested file
  boost::filesystem::path path =
      boost::filesystem::path(config_.doc_root) / std::string(req.target());

  if (req.target().back() == '/')
    path.append("index.html");

#ifdef _WIN32
  path.imbue(
      std::locale(std::locale(), new std::codecvt_utf8_utf16<wchar_t>()));
#endif
  // Attempt to open the file
  boost::beast::error_code ec;
  boost::beast::http::file_body::value_type body;
  body.open(path.string().c_str(), boost::beast::file_mode::scan, ec);

  // Handle the case where the file doesn't exist
  if (ec == boost::system::errc::no_such_file_or_directory)
    return SendResponse(Util::NotFound(req, req.target()));

  // Handle an unknown error
  if (ec)
    return SendResponse(Util::ServerError(req, ec.message()));

  // Cache the size since we need it after the move
  auto const size = body.size();

  // HEAD リクエスト
  if (req.method() == boost::beast::http::verb::head) {
    boost::beast::http::response<boost::beast::http::empty_body> res{
        boost::beast::http::status::ok, req.version()};
    res.set(boost::beast::http::field::server, BOOST_BEAST_VERSION_STRING);
    res.set(boost::beast::http::field::content_type,
            Util::MimeType(path.string()));
    res.content_length(size);
    res.keep_alive(req.keep_alive());
    return SendResponse(std::move(res));
  }

  // GET リクエスト
  boost::beast::http::response<boost::beast::http::file_body> res{
      std::piecewise_construct, std::make_tuple(std::move(body)),
      std::make_tuple(boost::beast::http::status::ok, req.version())};
  res.set(boost::beast::http::field::server, BOOST_BEAST_VERSION_STRING);
  res.set(boost::beast::http::field::content_type,
          Util::MimeType(path.string()));
  res.content_length(size);
  res.keep_alive(req.keep_alive());
  return SendResponse(std::move(res));
}

void P2PSession::OnWrite(boost::system::error_code ec,
                         std::size_t bytes_transferred,
                         bool close) {
  boost::ignore_unused(bytes_transferred);

  if (ec)
    return MOMO_BOOST_ERROR(ec, "write");

  if (close)
    return DoClose();

  res_ = nullptr;

  DoRead();
}

void P2PSession::DoClose() {
  boost::system::error_code ec;
  socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_send, ec);
}
