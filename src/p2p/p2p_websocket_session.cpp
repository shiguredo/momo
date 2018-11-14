#include "p2p_websocket_session.h"

#include <nlohmann/json.hpp>
#include "util.h"

using json = nlohmann::json;

// RAII を使ってエラーの時に ws_ と connection_ を reset する。
// ws_, connection_ と this で循環参照させてるので、処理を継続しない場合には ws_ と connection_ を reset しておかないとリークしてしまう。
struct P2PWebsocketSession::SafeWS {
    P2PWebsocketSession* p;
    SafeWS(P2PWebsocketSession* p) : p(p) {}
    ~SafeWS() {
        if (p != nullptr) {
            p->ws_.reset();
            p->connection_.reset();
        }
    }
    void ok() { p = nullptr; }
};

P2PWebsocketSession::P2PWebsocketSession(RTCManager* rtc_manager, ConnectionSettings conn_settings)
    : rtc_manager_(rtc_manager)
    , conn_settings_(conn_settings)
{
    RTC_LOG(LS_INFO) << __FUNCTION__;
}

P2PWebsocketSession::~P2PWebsocketSession()
{
    RTC_LOG(LS_INFO) << __FUNCTION__;
}

std::shared_ptr<P2PWebsocketSession> P2PWebsocketSession::make_shared(boost::asio::ip::tcp::socket socket, RTCManager* rtc_manager, ConnectionSettings conn_settings) {
    auto p = std::make_shared<P2PWebsocketSession>(rtc_manager, conn_settings);
    p->ws_ = std::make_shared<Websocket>(std::move(socket),
        std::bind(
            &P2PWebsocketSession::onRead,
            p->shared_from_this(),
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3),
        std::bind(
            &P2PWebsocketSession::onWrite,
            p->shared_from_this(),
            std::placeholders::_1,
            std::placeholders::_2));
    return p;
}

void P2PWebsocketSession::run(boost::beast::http::request<boost::beast::http::string_body> req)
{
    RTC_LOG(LS_INFO) << __FUNCTION__;
    doAccept(std::move(req));
}

void P2PWebsocketSession::doAccept(boost::beast::http::request<boost::beast::http::string_body> req)
{
    RTC_LOG(LS_INFO) << __FUNCTION__;
    // Accept the websocket handshake
    ws_->nativeSocket().async_accept(
        req,
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(
                &P2PWebsocketSession::onAccept,
                shared_from_this(),
                std::placeholders::_1)));
}

void P2PWebsocketSession::onAccept(boost::system::error_code ec)
{
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec;

    SafeWS sws(this);

    if (ec)
        return MOMO_BOOST_ERROR(ec, "Accept");

    // WebSocket での読み込みを開始
    ws_->startToRead();

    sws.ok();
}

void P2PWebsocketSession::onRead(boost::system::error_code ec, std::size_t bytes_transferred, std::string recv_string)
{
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec;

    boost::ignore_unused(bytes_transferred);

    SafeWS sws(this);

    if (ec == boost::beast::websocket::error::closed)
        return;

    if (ec)
        return MOMO_BOOST_ERROR(ec, "Read");

    json recv_message;

    RTC_LOG(LS_INFO) << __FUNCTION__ << ": recv_string=" << recv_string;

    try
    {
        recv_message = json::parse(recv_string);
    }
    catch (json::parse_error &e)
    {
        return;
    }

    std::string type;
    try
    {
      type = recv_message["type"];
    }
    catch (json::type_error &e)
    {
        return;
    }

    if (type == "offer")
    {
        std::string sdp;
        try
        {
            sdp = recv_message["sdp"];
        }
        catch (json::type_error &e)
        {
            return;
        }

        auto send = std::bind(
            [](std::shared_ptr<P2PWebsocketSession> session, std::string str) {
                session->ws_->sendText(str);
            },
            shared_from_this(),
            std::placeholders::_1);
        connection_ = std::make_shared<P2PConnection>(rtc_manager_, send);
        std::shared_ptr<RTCConnection> rtc_conn = connection_->getRTCConnection();
        rtc_conn->setOffer(sdp);
    }
    else if (type == "answer")
    {
        std::shared_ptr<P2PConnection> p2p_conn = connection_;
        if (!p2p_conn)
        {
            return;
        }
        std::string sdp;
        try
        {
            sdp = recv_message["sdp"];
        }
        catch (json::type_error &e)
        {
            return;
        }
        std::shared_ptr<RTCConnection> rtc_conn = p2p_conn->getRTCConnection();
        rtc_conn->setAnswer(sdp);
    }
    else if (type == "candidate")
    {
        std::shared_ptr<P2PConnection> p2p_conn = connection_;
        if (!p2p_conn)
        {
            return;
        }
        int sdp_mlineindex = 0;
        std::string sdp_mid, candidate;
        try
        {
            json ice = recv_message["ice"];
            sdp_mid = ice["sdpMid"];
            sdp_mlineindex = ice["sdpMLineIndex"];
            candidate = ice["candidate"];
        }
        catch (json::type_error &e)
        {
            return;
        }
        std::shared_ptr<RTCConnection> rtc_conn = p2p_conn->getRTCConnection();
        rtc_conn->addIceCandidate(sdp_mid, sdp_mlineindex, candidate);
    }
    else if (type == "close")
    {
        connection_ = nullptr;
    }
    else
    {
        return;
    }

    sws.ok();
}

void P2PWebsocketSession::onWrite(boost::system::error_code ec, std::size_t bytes_transferred)
{
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec;

    SafeWS sws(this);

    if (ec == boost::asio::error::operation_aborted)
        return;

    if (ec)
        return MOMO_BOOST_ERROR(ec, "Write");

    sws.ok();
}
