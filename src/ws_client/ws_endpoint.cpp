#include "ws_endpoint.h"

void WebSocketEndpoint::sendText(std::string message)
{
  RTC_LOG(LS_INFO) << __FUNCTION__  << " <<<< sendText text : " << message;
  websocketpp::lib::error_code error_code;
  _client->send(_con_ptr->get_handle(), message, websocketpp::frame::opcode::text, error_code);
  if (error_code)
  {
    RTC_LOG(LS_WARNING) << __FUNCTION__ << " sendText error: " 
                        << error_code.message();
  }
}

void WebSocketEndpoint::close()
{
  close(websocketpp::close::status::normal, "client_exit");
}

void WebSocketEndpoint::close(
        websocketpp::close::status::value code,
        std::string reason)
{
  _wsstate = WebSocketState::kWebSocketDisconnecting;
  websocketpp::lib::error_code error_code;
  _client->close(_con_ptr->get_handle(), code, reason, error_code);
  if (error_code) {
    RTC_LOG(LS_WARNING) << __FUNCTION__ << " close error: " 
                        << error_code.message();
  }
}

void WebSocketEndpoint::release()
{
  _listener = nullptr;
  close();
}

void WebSocketEndpoint::onOpen(WSPPHandle handle)
{
  RTC_LOG(LS_INFO) << __FUNCTION__;
  _wsstate = WebSocketState::kWebSocketConnected;
  if (_listener != nullptr) {
    _listener->onOpen();
  }
}

void WebSocketEndpoint::onClose(WSPPHandle handle)
{
  RTC_LOG(LS_INFO) << __FUNCTION__;
  _wsstate = WebSocketState::kWebSocketClosed;
  if (_listener != nullptr) {
    _listener->onClose();
  }
}

void WebSocketEndpoint::onFail(WSPPHandle handle)
{
  RTC_LOG(LS_INFO) << __FUNCTION__;
  _wsstate = WebSocketState::kWebSocketFailed;
  if (_listener != nullptr) {
    _listener->onFail();
  }
}

void WebSocketEndpoint::onMessage(WSPPHandle handle, WSPPMessagePtr msg_ptr)
{
  if (msg_ptr->get_opcode() == websocketpp::frame::opcode::text) {
    RTC_LOG(LS_INFO) << __FUNCTION__   << " >>>> onMessage text : "
                     << msg_ptr->get_payload();
    if (_listener != nullptr) {
      _listener->onMessage(msg_ptr->get_payload());
    }
  } else {
    RTC_LOG(LS_INFO) << __FUNCTION__   << " >>>> onMessage data : " 
                     << websocketpp::utility::to_hex(msg_ptr->get_payload());
  }
}
