#include "sora_session.h"

// Boost
#include <boost/beast/http/read.hpp>
#include <boost/beast/version.hpp>

// nlohman/json
#include <nlohmann/json.hpp>

#include "util.h"

using json = nlohmann::json;

SoraSession::SoraSession(boost::asio::ip::tcp::socket socket,
                         std::shared_ptr<SoraClient> client,
                         RTCManager* rtc_manager,
                         ConnectionSettings conn_settings)
    : socket_(std::move(socket)),
      client_(client),
      rtc_manager_(rtc_manager),
      conn_settings_(conn_settings) {}

void SoraSession::Run() {
  DoRead();
}

void SoraSession::DoRead() {
  // Make the request empty before reading,
  // otherwise the operation behavior is undefined.
  req_ = {};

  // Read a request
  boost::beast::http::async_read(
      socket_, buffer_, req_,
      std::bind(&SoraSession::OnRead, shared_from_this(), std::placeholders::_1,
                std::placeholders::_2));
}

void SoraSession::OnRead(boost::system::error_code ec,
                         std::size_t bytes_transferred) {
  boost::ignore_unused(bytes_transferred);

  // This means they closed the connection
  if (ec == boost::beast::http::error::end_of_stream)
    return DoClose();

  if (ec)
    return MOMO_BOOST_ERROR(ec, "read");

  if (req_.method() == boost::beast::http::verb::get) {
    if (req_.target() == "/connect/status") {
      std::string state =
          Util::IceConnectionStateToString(client_->GetRTCConnectionState());
      json json_message = {{"state", state}};
      SendResponse(CreateOKWithJSON(req_, std::move(json_message)));
    } else if (req_.target() == "/mute/status") {
      std::shared_ptr<RTCConnection> rtc_conn = client_->GetRTCConnection();
      if (rtc_conn) {
        json json_message = {{"audio", !rtc_conn->IsAudioEnabled()},
                             {"video", !rtc_conn->IsVideoEnabled()}};
        SendResponse(CreateOKWithJSON(req_, std::move(json_message)));
      } else {
        SendResponse(Util::ServerError(req_, "Invalid RTC Connection"));
      }
    } else {
      SendResponse(Util::BadRequest(req_, "Invalid Request"));
    }
  } else if (req_.method() == boost::beast::http::verb::post) {
    if (req_.target() == "/connect") {
      client_->Connect();
      json json_message = {{"result", true}};
      SendResponse(CreateOKWithJSON(req_, std::move(json_message)));
    } else if (req_.target() == "/close") {
      client_->Close();
      json json_message = {{"result", true}};
      SendResponse(CreateOKWithJSON(req_, std::move(json_message)));
    } else if (req_.target() == "/mute") {
      json recv_json;
      try {
        recv_json = json::parse(req_.body());
      } catch (json::parse_error& e) {
        SendResponse(Util::BadRequest(req_, "Invalid JSON"));
        return;
      }

      std::shared_ptr<RTCConnection> rtc_conn = client_->GetRTCConnection();
      if (!rtc_conn) {
        SendResponse(Util::ServerError(req_, "Create RTC Connection Failed"));
        return;
      }
      try {
        bool audioMute = recv_json["audio"].get<bool>();
        rtc_conn->SetAudioEnabled(!audioMute);
      } catch (json::type_error& e) {
      }

      try {
        bool videoMute = recv_json["video"].get<bool>();
        rtc_conn->SetVideoEnabled(!videoMute);
      } catch (json::type_error& e) {
      }

      json json_message = {{"audio", !rtc_conn->IsAudioEnabled()},
                           {"video", !rtc_conn->IsVideoEnabled()}};
      SendResponse(CreateOKWithJSON(req_, std::move(json_message)));
    } else {
      SendResponse(Util::BadRequest(req_, "Invalid Request"));
    }
  } else {
    SendResponse(Util::BadRequest(req_, "Invalid Method"));
  }
}

void SoraSession::OnWrite(boost::system::error_code ec,
                          std::size_t bytes_transferred,
                          bool close) {
  boost::ignore_unused(bytes_transferred);

  if (ec)
    return MOMO_BOOST_ERROR(ec, "write");

  if (close) {
    // This means we should close the connection, usually because
    // the response indicated the "Connection: close" semantic.
    return DoClose();
  }

  // We're done with the response so delete it
  res_ = nullptr;

  // Read another request
  DoRead();
}

void SoraSession::DoClose() {
  // Send a TCP shutdown
  boost::system::error_code ec;
  socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_send, ec);

  // At this point the connection is closed gracefully
}

boost::beast::http::response<boost::beast::http::string_body>
SoraSession::CreateOKWithJSON(
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
