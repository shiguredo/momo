#ifndef SORA_V4L2_DEVICE_H_
#define SORA_V4L2_DEVICE_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sora {

struct V4L2Device;
struct V4L2FormatDescription;
struct V4L2FrameSize;
struct V4L2FrameInterval;
struct V4L2IntervalsAtSize;

// VIDIOC_QUERYCAP
struct V4L2Device {
  int index;
  std::string path;

  // デバイス名
  std::string card;
  // 表示名
  std::string bus_info;

  std::vector<V4L2FormatDescription> format_descriptions;
};

// VIDIOC_ENUM_FMT
struct V4L2FormatDescription {
  int index;
  uint32_t pixel_format;
  std::string description;
  std::vector<V4L2FrameSize> frame_sizes;
};

// フレームサイズの種類
enum class V4L2FrameSizeType {
  DISCRETE,
  STEPWISE,
  CONTINUOUS,
};

// フレームレート(フレーム間隔)の種類
enum class V4L2FrameIntervalType {
  DISCRETE,
  STEPWISE,
  CONTINUOUS,
};

// VIDIOC_ENUM_FRAMESIZES の結果を汎用化した構造体
struct V4L2FrameSize {
  int index;  // DISCRETE の時のみ意味がある
  V4L2FrameSizeType type;

  // DISCRETE の場合
  int width = 0;
  int height = 0;

  // STEPWISE/CONTINUOUS の場合
  int min_width = 0;
  int max_width = 0;
  int step_width = 0;  // CONTINUOUS の場合は 1 相当
  int min_height = 0;
  int max_height = 0;
  int step_height = 0;  // CONTINUOUS の場合は 1 相当

  // 対応するフレームレート情報
  // DISCRETE の場合はこちらを使用
  std::vector<V4L2FrameInterval> intervals;
  // STEPWISE/CONTINUOUS の場合はこちらを使用（代表サイズ: 最小/中央値/最大）
  std::vector<V4L2IntervalsAtSize> intervals_at_sizes;
};

// VIDIOC_ENUM_FRAMEINTERVALS の結果を汎用化した構造体
struct V4L2FrameInterval {
  int index;  // DISCRETE の時のみ意味がある
  V4L2FrameIntervalType type;

  // DISCRETE の場合 (interval = numerator/denominator 秒)
  int numerator = 0;
  int denominator = 0;

  // STEPWISE/CONTINUOUS の場合
  int min_numerator = 0;
  int min_denominator = 0;
  int max_numerator = 0;
  int max_denominator = 0;
  int step_numerator = 0;    // CONTINUOUS の場合は 1 相当
  int step_denominator = 0;  // CONTINUOUS の場合は 1 相当
};

// 範囲サイズに対して、代表サイズごとのフレームレート情報
enum class V4L2SizePoint { MIN, MID, MAX };
struct V4L2IntervalsAtSize {
  V4L2SizePoint point;
  int width = 0;
  int height = 0;
  std::vector<V4L2FrameInterval> intervals;
};

std::optional<std::vector<V4L2Device>> EnumV4L2CaptureDevices();
std::string FormatV4L2Devices(const std::vector<V4L2Device>& devices);
std::string FormatV4L2Device(const V4L2Device& device, int indent = 0);
std::string FormatV4L2FormatDescription(
    const V4L2FormatDescription& format_desc,
    int indent = 0);
std::string FormatV4L2FrameSize(const V4L2FrameSize& frame_size,
                                int indent = 0);
std::string FormatV4L2IntervalsAtSize(const V4L2IntervalsAtSize& at,
                                      int indent = 0);

}  // namespace sora

#endif
