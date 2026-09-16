#ifndef ZLIB_HELPER_H_
#define ZLIB_HELPER_H_

#include <optional>
#include <string>

// zlib
#include <zlib.h>

class ZlibHelper {
 public:
  // DataChannel 受信の展開後上限。正当な signaling / stats は数十 KB 〜数 MB 程度で、
  // libwebrtc の受信バッファ (5 MiB) と圧縮率の伸びしろを見ても 16 MiB あれば足りる。
  // zip 爆弾の無制限倍増はここで打ち切る。
  static constexpr size_t kMaxUncompressedSize = 16 * 1024 * 1024;

  static std::string Compress(const std::string& input,
                              int level = Z_DEFAULT_COMPRESSION) {
    return Compress((const uint8_t*)input.data(), input.size(), level);
  }

  static std::string Compress(const uint8_t* input_buf,
                              size_t input_size,
                              int level = Z_DEFAULT_COMPRESSION) {
    std::string output;
    output.resize(16 * 1024);
    uLongf output_size;
    while (true) {
      output_size = output.size();
      int ret = compress2((Bytef*)output.data(), &output_size, input_buf,
                          input_size, level);
      if (ret == Z_BUF_ERROR) {
        output.resize(output.size() * 2);
        continue;
      }
      if (ret != Z_OK) {
        throw std::exception();
      }
      break;
    }
    output.resize(output_size);
    return output;
  }

  static std::optional<std::string> Uncompress(const std::string& input) {
    return Uncompress((const uint8_t*)input.data(), input.size());
  }

  static std::optional<std::string> Uncompress(const uint8_t* input_buf,
                                               size_t input_size) {
    std::string output;
    output.resize(16 * 1024);
    uLongf output_size;
    while (true) {
      output_size = output.size();
      int ret = uncompress((Bytef*)output.data(), &output_size, input_buf,
                           input_size);
      if (ret == Z_BUF_ERROR) {
        // 次の倍増が上限を超える場合は拒否し、無制限拡張を防ぐ
        if (output.size() >= kMaxUncompressedSize ||
            output.size() > kMaxUncompressedSize / 2) {
          return std::nullopt;
        }
        output.resize(output.size() * 2);
        continue;
      }
      if (ret != Z_OK) {
        return std::nullopt;
      }
      break;
    }
    output.resize(output_size);
    return output;
  }
};

#endif
