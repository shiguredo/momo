#include "p2p_session.h"

#include <boost/filesystem.hpp>
#include "util.h"

P2PSession::P2PSession(
    boost::asio::ip::tcp::socket socket,
    std::shared_ptr<std::string const> const& doc_root,
    RTCManager* rtc_manager,
    ConnectionSettings conn_settings)
    : socket_(std::move(socket))
    , strand_(socket_.get_executor())
    , doc_root_(doc_root)
    , rtc_manager_(rtc_manager)
    , conn_settings_(conn_settings)
{
}

// Start the asynchronous operation
void P2PSession::run()
{
    doRead();
}

void P2PSession::doRead()
{
    // Make the request empty before reading,
    // otherwise the operation behavior is undefined.
    req_ = {};

    // Read a request
    boost::beast::http::async_read(socket_, buffer_, req_,
        boost::asio::bind_executor(
            strand_,
            std::bind(
                &P2PSession::onRead,
                shared_from_this(),
                std::placeholders::_1,
                std::placeholders::_2)));
}

void P2PSession::onRead(
    boost::system::error_code ec,
    std::size_t bytes_transferred)
{
    boost::ignore_unused(bytes_transferred);

    // 接続が切られた
    if (ec == boost::beast::http::error::end_of_stream)
        return doClose();

    if (ec)
        return MOMO_BOOST_ERROR(ec, "read");

    // WebSocket の upgrade リクエスト
    if (req_.target() == "/ws")
    {
      if (boost::beast::websocket::is_upgrade(req_))
      {
          P2PWebsocketSession::make_shared(std::move(socket_), rtc_manager_, conn_settings_)->run(std::move(req_));
          return;
      }
      else
      {
          sendResponse(Util::badRequest(std::move(req_), "Not upgrade request"));
          return;
      }
    }

    handleRequest();
}

void P2PSession::handleRequest() {
    boost::beast::http::request<boost::beast::http::string_body> req(std::move(req_));

    // Make sure we can handle the method
    if( req.method() != boost::beast::http::verb::get &&
        req.method() != boost::beast::http::verb::head)
        return sendResponse(Util::badRequest(std::move(req), "Unknown HTTP-method"));

    // Request path must be absolute and not contain "..".
    if( req.target().empty() ||
        req.target()[0] != '/' ||
        req.target().find("..") != boost::beast::string_view::npos)
        return sendResponse(Util::badRequest(req, "Illegal request-target"));

    // Build the path to the requested file
    boost::filesystem::path path = boost::filesystem::path(*doc_root_) / std::string(req.target());

    if (req.target().back() == '/')
        path.append("index.html");

    // Attempt to open the file
    boost::beast::error_code ec;
    boost::beast::http::file_body::value_type body;
    body.open(path.c_str(), boost::beast::file_mode::scan, ec);

    // Handle the case where the file doesn't exist
    if(ec == boost::system::errc::no_such_file_or_directory)
        return sendResponse(Util::notFound(req, req.target()));

    // Handle an unknown error
    if(ec)
        return sendResponse(Util::serverError(req, ec.message()));

    // Cache the size since we need it after the move
    auto const size = body.size();

    // HEAD リクエスト
    if(req.method() == boost::beast::http::verb::head)
    {
        boost::beast::http::response<boost::beast::http::empty_body> res{boost::beast::http::status::ok, req.version()};
        res.set(boost::beast::http::field::server, BOOST_BEAST_VERSION_STRING);
        res.set(boost::beast::http::field::content_type, Util::mimeType(path.string()));
        res.content_length(size);
        res.keep_alive(req.keep_alive());
        return sendResponse(std::move(res));
    }

    // GET リクエスト
    boost::beast::http::response<boost::beast::http::file_body> res{
        std::piecewise_construct,
        std::make_tuple(std::move(body)),
        std::make_tuple(boost::beast::http::status::ok, req.version())};
    res.set(boost::beast::http::field::server, BOOST_BEAST_VERSION_STRING);
    res.set(boost::beast::http::field::content_type, Util::mimeType(path.string()));
    res.content_length(size);
    res.keep_alive(req.keep_alive());
    return sendResponse(std::move(res));
}

void P2PSession::onWrite(
    boost::system::error_code ec,
    std::size_t bytes_transferred,
    bool close)
{
    boost::ignore_unused(bytes_transferred);

    if (ec)
        return MOMO_BOOST_ERROR(ec, "write");

    if (close)
        return doClose();

    res_ = nullptr;

    doRead();
}

void P2PSession::doClose()
{
    boost::system::error_code ec;
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_send, ec);
}
