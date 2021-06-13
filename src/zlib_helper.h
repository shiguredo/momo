#ifndef ZLIB_HELPER_H_
#define ZLIB_HELPER_H_

#include <string>

// zlib
#include <zlib.h>

class ZlibHelper {
 public:
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

  static std::string Uncompress(const std::string& input) {
    return Uncompress((const uint8_t*)input.data(), input.size());
  }

  static std::string Uncompress(const uint8_t* input_buf, size_t input_size) {
    std::string output;
    output.resize(16 * 1024);
    uLongf output_size;
    while (true) {
      output_size = output.size();
      int ret = uncompress((Bytef*)output.data(), &output_size, input_buf,
                           input_size);
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
};

#endif