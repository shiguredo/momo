#ifndef URL_PARTS_H_
#define URL_PARTS_H_

#include <string>

/*
適当な URL パーサ

Boost.Beast の URL パーサはまだ [PR の段階](https://github.com/boostorg/beast/pull/1231) なので、自前で適当に作っておく。
*/
struct URLParts {
  std::string scheme;
  std::string user_pass;
  std::string host;
  std::string port;
  std::string path_query_fragment;

  // 適当 URL パース
  // scheme://[user_pass@]host[:port][/path_query_fragment]
  static bool parse(std::string url, URLParts& parts) {
    auto n = url.find("://");
    if (n == std::string::npos) {
      return false;
    }
    parts.scheme = url.substr(0, n);

    n += 3;
    auto m = url.find('/', n);
    std::string user_pass_host_port;
    if (m == std::string::npos) {
      user_pass_host_port = url.substr(n);
      parts.path_query_fragment = "";
    } else {
      user_pass_host_port = url.substr(n, m - n);
      parts.path_query_fragment = url.substr(m);
    }

    n = 0;
    m = user_pass_host_port.find('@');
    std::string host_port;
    if (m == std::string::npos) {
      parts.user_pass = "";
      host_port = std::move(user_pass_host_port);
    } else {
      parts.user_pass = user_pass_host_port.substr(n, m - n);
      host_port = user_pass_host_port.substr(m + 1);
    }

    n = 0;
    m = host_port.find(':');
    if (m == std::string::npos) {
      parts.host = std::move(host_port);
      parts.port = "";
    } else {
      parts.host = host_port.substr(n, m - n);
      parts.port = host_port.substr(m + 1);
    }

    return true;
  }
};

#endif  // URL_PARTS_H_
