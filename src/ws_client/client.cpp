#include "client.h"

WebSocketClient::WebSocketClient()
{
  using websocketpp::log::alevel;
  _client.clear_access_channels(alevel::all);

  _client.init_asio();
  _client.start_perpetual();

  _client.set_tls_init_handler(websocketpp::lib::bind(&WebSocketClient::onTLSInit, this, websocketpp::lib::placeholders::_1));

  _thread.reset(new websocketpp::lib::thread(&WSPPClient::run, &_client));
}

WebSocketClient::~WebSocketClient()
{
  _client.stop_perpetual();
  _thread->join();
}

std::shared_ptr<WebSocketEndpoint> WebSocketClient::createEndpoint(
        const std::string url, WebSocketEventListener *listener)
{
  using websocketpp::lib::placeholders::_1;
  using websocketpp::lib::placeholders::_2;
  using websocketpp::lib::bind;

  websocketpp::lib::error_code error_code;
  WSPPClient::connection_ptr con_ptr = _client.get_connection(url, error_code);
  if (error_code)
  {
    RTC_LOG(LS_WARNING) << __FUNCTION__ << " connect initialization error: "
                        << error_code.message();
    return nullptr;
  }

  std::shared_ptr<WebSocketEndpoint> endpoint =
          std::make_shared<WebSocketEndpoint>(&_client, con_ptr, listener);

  con_ptr->set_message_handler(bind(&WebSocketEndpoint::onMessage, endpoint, _1, _2));
  con_ptr->set_open_handler(bind(&WebSocketEndpoint::onOpen, endpoint, _1));
  con_ptr->set_close_handler(bind(&WebSocketEndpoint::onClose, endpoint, _1));
  con_ptr->set_fail_handler(bind(&WebSocketEndpoint::onFail, endpoint, _1));

  _client.connect(con_ptr);
  return endpoint;
}

WSPPContextPtr WebSocketClient::onTLSInit(WSPPHandle handle)
{
  WSPPContextPtr ctx = std::make_shared<boost::asio::ssl::context>(boost::asio::ssl::context::tlsv12);
  try
  {
    ctx->set_options(boost::asio::ssl::context::default_workarounds |
                     boost::asio::ssl::context::no_sslv2 |
                     boost::asio::ssl::context::no_sslv3 |
                     boost::asio::ssl::context::single_dh_use);
  }
  catch (std::exception &e)
  {
    RTC_LOG(LS_WARNING) << __FUNCTION__ << " onTLSInit error:" << e.what();
  }
  return ctx;
}
