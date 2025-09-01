#ifndef SORA_V4L2_DEVICE_H_
#define SORA_V4L2_DEVICE_H_

#include <optional>
#include <string>
#include <vector>

namespace sora {

struct V4L2Device;
struct V4L2FormatDescription;
struct V4L2DiscreteFrameSize;
struct V4L2DiscreteInterval;

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
  std::vector<V4L2DiscreteFrameSize> frame_sizes;
};

// VIDIOC_ENUM_FRAMESIZES が V4L2_FRMSIZE_TYPE_DISCRETE だった場合の構造体
struct V4L2DiscreteFrameSize {
  int index;
  int width;
  int height;
  std::vector<V4L2DiscreteInterval> intervals;
};

// VIDIOC_ENUM_FRAMEINTERVALS が V4L2_FRMIVAL_TYPE_DISCRETE だった場合の構造体
struct V4L2DiscreteInterval {
  int index;
  int numerator;
  int denominator;
};

std::optional<std::vector<V4L2Device>> EnumV4L2CaptureDevices();
std::string FormatV4L2Devices(const std::vector<V4L2Device>& devices);
std::string FormatV4L2Device(const V4L2Device& device, int indent = 0);
std::string FormatV4L2FormatDescription(
    const V4L2FormatDescription& format_desc,
    int indent = 0);
std::string FormatV4L2DiscreteFrameSize(const V4L2DiscreteFrameSize& frame_size,
                                        int indent = 0);

}  // namespace sora

#endif
