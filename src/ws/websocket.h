#ifndef WS_WEBSOCKET_H_
#define WS_WEBSOCKET_H_

#include <boost/beast.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <boost/asio.hpp>
#include <boost/asio/ssl/stream.hpp>
#include <functional>
#include <memory>

// ずっと Read しつつ、書き込みが来たら送信してくれる WebSocket
// サーバ用、クライアント用どちらからでも使える。
// 任意のスレッドから sendText を呼ぶことで書き込みができる。
//
// 接続の確立に関しては、nativeSocket() および nativeSecureSocket() 関数を使って自前で行うこと。
class Websocket : public std::enable_shared_from_this<Websocket>
{
public:
    typedef boost::beast::websocket::stream<boost::asio::ip::tcp::socket> websocket_t;
    typedef boost::beast::websocket::stream<boost::asio::ssl::stream<boost::asio::ip::tcp::socket>> ssl_websocket_t;
    typedef std::function<void (boost::system::error_code ec, std::size_t bytes_transferred, std::string text)> read_callback_t;
    typedef std::function<void (boost::system::error_code ec, std::size_t bytes_transferred)> write_callback_t;

private:
    std::unique_ptr<websocket_t> ws_;
    std::unique_ptr<ssl_websocket_t> wss_;

    read_callback_t on_read_;
    write_callback_t on_write_;

    boost::asio::strand<boost::asio::io_context::executor_type> strand_;

    boost::beast::multi_buffer read_buffer_;
    boost::beast::multi_buffer write_buffer_;

public:
    // 非SSL
    Websocket(boost::asio::io_context& ioc, read_callback_t on_read, write_callback_t on_write);
    // SSL
    Websocket(boost::asio::io_context& ioc, boost::asio::ssl::context ssl_ctx, read_callback_t on_read, write_callback_t on_write);
    // 非SSL + ソケット直接
    Websocket(boost::asio::ip::tcp::socket socket, read_callback_t on_read, write_callback_t on_write);

    bool isSSL() const;

    websocket_t& nativeSocket();
    ssl_websocket_t& nativeSecureSocket();

    boost::asio::strand<boost::asio::io_context::executor_type>& strand();

public:
    // Websocket からの読み込みを開始する。
    // これを呼ばなくても、最初の書き込みを行ったら勝手に読み込みも始まるが、
    // 最初の読み込みを明示的に行いたい場合に呼ぶ。
    void startToRead();

private:
    void doRead();
    void onRead(boost::system::error_code ec, std::size_t bytes_transferred);

public:
    void sendText(std::string text);

private:
    void doSendText(std::string text);
    void doWrite();

    void onWrite(boost::system::error_code ec, std::size_t bytes_transferred);
};

#endif // WS_WEBSOCKET_H_
