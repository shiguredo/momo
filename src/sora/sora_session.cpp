#include "sora_session.h"

#include <boost/beast/http/read.hpp>
#include <boost/beast/version.hpp>
#include <nlohmann/json.hpp>

#include "util.h"

using json = nlohmann::json;

SoraSession::SoraSession(boost::asio::ip::tcp::socket socket,
                         RTCManager* rtc_manager,
                         std::shared_ptr<SoraWebsocketClient> ws_client,
                         ConnectionSettings conn_settings)
    : socket_(std::move(socket)),
      strand_(socket_.get_executor()),
      rtc_manager_(rtc_manager),
      ws_client_(ws_client),
      conn_settings_(conn_settings) {}

void SoraSession::run() {
  doRead();
}

void SoraSession::doRead() {
  // Make the request empty before reading,
  // otherwise the operation behavior is undefined.
  req_ = {};

  // Read a request
  boost::beast::http::async_read(
      socket_, buffer_, req_,
      boost::asio::bind_executor(
          strand_, std::bind(&SoraSession::onRead, shared_from_this(),
                             std::placeholders::_1, std::placeholders::_2)));
}

void SoraSession::onRead(boost::system::error_code ec,
                         std::size_t bytes_transferred) {
  boost::ignore_unused(bytes_transferred);

  // This means they closed the connection
  if (ec == boost::beast::http::error::end_of_stream)
    return doClose();

  if (ec)
    return MOMO_BOOST_ERROR(ec, "read");

  if (req_.method() == boost::beast::http::verb::get) {
    if (req_.target() == "/connect/status") {
      std::string state =
          Util::iceConnectionStateToString(ws_client_->getRTCConnectionState());
      json json_message = {{"state", state}};
      sendResponse(createOKwithJson(req_, std::move(json_message)));
    } else if (req_.target() == "/mute/status") {
      std::shared_ptr<RTCConnection> rtc_conn = ws_client_->getRTCConnection();
      if (rtc_conn) {
        json json_message = {{"audio", !rtc_conn->isAudioEnabled()},
                             {"video", !rtc_conn->isVideoEnabled()}};
        sendResponse(createOKwithJson(req_, std::move(json_message)));
      } else {
        sendResponse(Util::serverError(req_, "Invalid RTC Connection"));
      }
    } else {
      sendResponse(Util::badRequest(req_, "Invalid Request"));
    }
  } else if (req_.method() == boost::beast::http::verb::post) {
    if (req_.target() == "/connect") {
      json json_message = {{"result", ws_client_->connect()}};
      sendResponse(createOKwithJson(req_, std::move(json_message)));
    } else if (req_.target() == "/close") {
      json json_message = {{"result", true}};
      sendResponse(createOKwithJson(req_, std::move(json_message)));
    } else if (req_.target() == "/mute") {
      json recv_json;
      try {
        recv_json = json::parse(req_.body());
      } catch (json::parse_error& e) {
        sendResponse(Util::badRequest(req_, "Invalid JSON"));
        return;
      }

      std::shared_ptr<RTCConnection> rtc_conn = ws_client_->getRTCConnection();
      if (!rtc_conn) {
        sendResponse(Util::serverError(req_, "Create RTC Connection Failed"));
        return;
      }
      try {
        bool audioMute = recv_json["audio"].get<bool>();
        rtc_conn->setAudioEnabled(!audioMute);
      } catch (json::type_error& e) {
      }

      try {
        bool videoMute = recv_json["video"].get<bool>();
        rtc_conn->setVideoEnabled(!videoMute);
      } catch (json::type_error& e) {
      }

      json json_message = {{"audio", !rtc_conn->isAudioEnabled()},
                           {"video", !rtc_conn->isVideoEnabled()}};
      sendResponse(createOKwithJson(req_, std::move(json_message)));
    } else {
      sendResponse(Util::badRequest(req_, "Invalid Request"));
    }
  } else {
    sendResponse(Util::badRequest(req_, "Invalid Method"));
  }
}

void SoraSession::onWrite(boost::system::error_code ec,
                          std::size_t bytes_transferred,
                          bool close) {
  boost::ignore_unused(bytes_transferred);

  if (ec)
    return MOMO_BOOST_ERROR(ec, "write");

  if (close) {
    // This means we should close the connection, usually because
    // the response indicated the "Connection: close" semantic.
    return doClose();
  }

  // We're done with the response so delete it
  res_ = nullptr;

  // Read another request
  doRead();
}

void SoraSession::doClose() {
  // Send a TCP shutdown
  boost::system::error_code ec;
  socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_send, ec);

  // At this point the connection is closed gracefully
}

boost::beast::http::response<boost::beast::http::string_body>
SoraSession::createOKwithJson(
    const boost::beast::http::request<boost::beast::http::string_body>& req,
    json json_message) {
  boost::beast::http::response<boost::beast::http::string_body> res{
      boost::beast::http::status::ok, 11};
  res.set(boost::beast::http::field::server, BOOST_BEAST_VERSION_STRING);
  res.set(boost::beast::http::field::content_type, "application/json");
  res.keep_alive(req.keep_alive());
  res.body() = json_message.dump();
  res.prepare_payload();

  return res;
}
