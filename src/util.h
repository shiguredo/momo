#ifndef RTC_UTIL_H_
#define RTC_UTIL_H_

#include <boost/beast/core/string.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/message.hpp>
#include "api/peerconnectioninterface.h"
#include "connection_settings.h"

class Util
{
  public:
    static void parseArgs(int argc, char *argv[], bool &is_daemon,
                          bool &use_p2p, bool &use_sora,
                          int &log_level, ConnectionSettings &cs);
    static std::string generateRundomChars();
    static std::string generateRundomChars(size_t length);
    static std::string iceConnectionStateToString(
            webrtc::PeerConnectionInterface::IceConnectionState state) ;

    // MIME type をファイル名の拡張子から調べる
    static boost::beast::string_view mimeType(boost::beast::string_view path);

    // エラーレスポンスをいい感じに作る便利関数
    static boost::beast::http::response<boost::beast::http::string_body> badRequest(const boost::beast::http::request<boost::beast::http::string_body>& req, boost::beast::string_view why);
    static boost::beast::http::response<boost::beast::http::string_body> notFound(const boost::beast::http::request<boost::beast::http::string_body>& req, boost::beast::string_view target);
    static boost::beast::http::response<boost::beast::http::string_body> serverError(const boost::beast::http::request<boost::beast::http::string_body>& req, boost::beast::string_view what);
};

// boost::system::error_code のエラーをいい感じに出力するマクロ
//
// if (ec)
//   return MOMO_BOOST_ERROR(ec, "onRead")
//
// のように、return と組み合わせて使える。
#define MOMO_BOOST_ERROR(ec, what) ([&ec]{ RTC_LOG(LS_ERROR) << __FUNCTION__ << " " what ": " << ec.message(); }())

#endif
