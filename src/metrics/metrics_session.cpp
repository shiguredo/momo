#include "metrics_session.h"

// Boost
#include <boost/beast/core/error.hpp>
#include <boost/beast/http/empty_body.hpp>
#include <boost/beast/http/error.hpp>
#include <boost/beast/http/read.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/version.hpp>
#include <boost/json.hpp>

#ifdef _WIN32
#include <codecvt>
#endif

#include "momo_version.h"
#include "util.h"

MetricsSession::MetricsSession(boost::asio::io_context& ioc,
                               boost::asio::ip::tcp::socket socket,
                               RTCManager* rtc_manager,
                               std::shared_ptr<StatsCollector> stats_collector,
                               MetricsSessionConfig config)
    : ioc_(ioc),
      socket_(std::move(socket)),
      strand_(socket_.get_executor()),
      rtc_manager_(rtc_manager),
      stats_collector_(stats_collector),
      config_(std::move(config)) {}

// Start the asynchronous operation
void MetricsSession::Run() {
  DoRead();
}

void MetricsSession::DoRead() {
  // Make the request empty before reading,
  // otherwise the operation behavior is undefined.
  req_ = {};

  // Read a request
  boost::beast::http::async_read(
      socket_, buffer_, req_,
      boost::asio::bind_executor(
          strand_, std::bind(&MetricsSession::OnRead, shared_from_this(),
                             std::placeholders::_1, std::placeholders::_2)));
}

void MetricsSession::OnRead(boost::system::error_code ec,
                            std::size_t bytes_transferred) {
  boost::ignore_unused(bytes_transferred);

  // 接続が切られた
  if (ec == boost::beast::http::error::end_of_stream)
    return DoClose();

  if (ec)
    return MOMO_BOOST_ERROR(ec, "read");

  if (req_.method() == boost::beast::http::verb::get) {
    if (req_.target() == "/metrics") {
      std::shared_ptr<MetricsSession> self(shared_from_this());
      stats_collector_->GetStats(
          [self](
              const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) {
            std::string stats = report ? report->ToJson() : "[]";
            boost::json::value json_message = {
                {"version", MomoVersion::GetClientName()},
                {"libwebrtc", MomoVersion::GetLibwebrtcName()},
                {"environment", MomoVersion::GetEnvironmentName()},
                {"stats", boost::json::parse(stats)}};

            self->SendResponse(
                CreateOKWithJSON(self->req_, std::move(json_message)));
          });
    } else {
      SendResponse(Util::NotFound(req_, req_.target()));
    }
  } else {
    SendResponse(Util::BadRequest(req_, "Invalid Method"));
  }
}

void MetricsSession::OnWrite(boost::system::error_code ec,
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

void MetricsSession::DoClose() {
  boost::system::error_code ec;
  socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_send, ec);
}

boost::beast::http::response<boost::beast::http::string_body>
MetricsSession::CreateOKWithJSON(
    const boost::beast::http::request<boost::beast::http::string_body>& req,
    boost::json::value json_message) {
  boost::beast::http::response<boost::beast::http::string_body> res{
      boost::beast::http::status::ok, 11};
  res.set(boost::beast::http::field::server, BOOST_BEAST_VERSION_STRING);
  res.set(boost::beast::http::field::content_type, "application/json");
  res.keep_alive(req.keep_alive());
  res.body() = boost::json::serialize(json_message);
  res.prepare_payload();

  return res;
}
