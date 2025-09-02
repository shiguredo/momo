#include "sora/v4l2/v4l2_device.h"

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

// linux
#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "media/base/video_common.h"  // webrtc::GetFourccName

namespace sora {

std::optional<std::vector<V4L2Device>> EnumV4L2CaptureDevices() {
  DIR* dir = opendir("/sys/class/video4linux");
  if (dir == nullptr) {
    std::cerr << "Could not open /sys/class/video4linux: " << strerror(errno)
              << " (" << errno << ")" << std::endl;
    return std::nullopt;
  }

  // ビデオデバイスの列挙
  std::vector<V4L2Device> candidate_devices;
  struct dirent* entry;
  while ((entry = readdir(dir)) != NULL) {
    if (strncmp(entry->d_name, "video", 5) != 0) {
      continue;
    }
    V4L2Device device;
    // videoXX の XX だけを取り出してインデックスにする
    // 単に文字列としてソートする時に video10 が video2 より前に来てしまうため
    int r = sscanf(entry->d_name, "video%d", &device.index);
    if (r != 1) {
      continue;
    }
    device.path = "/dev/" + std::string(entry->d_name);
    candidate_devices.push_back(device);
  }
  closedir(dir);

  std::sort(candidate_devices.begin(), candidate_devices.end(),
            [](const V4L2Device& a, const V4L2Device& b) {
              return a.index < b.index;
            });

  std::vector<V4L2Device> enabled_devices;
  for (const auto& cand_device : candidate_devices) {
    int fd = open(cand_device.path.c_str(), O_RDONLY);
    if (fd < 0) {
      continue;
    }

    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) != 0) {
      close(fd);
      continue;
    }
    // ビデオキャプチャーが可能なデバイスだけを対象とする
    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
      close(fd);
      continue;
    }
    V4L2Device device = cand_device;
    device.bus_info = (const char*)cap.bus_info;
    device.card = (const char*)cap.card;

    // フォーマットの列挙
    struct v4l2_fmtdesc fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.index = 0;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0) {
      V4L2FormatDescription format_desc;
      format_desc.index = fmt.index;
      format_desc.pixel_format = fmt.pixelformat;
      format_desc.description = (const char*)fmt.description;

      // 各フォーマットがサポートしている解像度の列挙
      struct v4l2_frmsizeenum frmsize;
      memset(&frmsize, 0, sizeof(frmsize));
      frmsize.index = 0;
      frmsize.pixel_format = fmt.pixelformat;

      while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
        V4L2FrameSize frame_size;
        frame_size.index = frmsize.index;

        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
          frame_size.type = V4L2FrameSizeType::DISCRETE;
          frame_size.width = frmsize.discrete.width;
          frame_size.height = frmsize.discrete.height;
        } else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
          frame_size.type = V4L2FrameSizeType::STEPWISE;
          frame_size.min_width = frmsize.stepwise.min_width;
          frame_size.max_width = frmsize.stepwise.max_width;
          frame_size.step_width = frmsize.stepwise.step_width;
          frame_size.min_height = frmsize.stepwise.min_height;
          frame_size.max_height = frmsize.stepwise.max_height;
          frame_size.step_height = frmsize.stepwise.step_height;
        } else if (frmsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {
          frame_size.type = V4L2FrameSizeType::CONTINUOUS;
          frame_size.min_width = frmsize.stepwise.min_width;
          frame_size.max_width = frmsize.stepwise.max_width;
          frame_size.step_width = 1;
          frame_size.min_height = frmsize.stepwise.min_height;
          frame_size.max_height = frmsize.stepwise.max_height;
          frame_size.step_height = 1;
        }

        // 各フォーマットの各解像度がサポートしているフレームレートの列挙
        auto enum_intervals = [&](int w, int h,
                                  V4L2IntervalsAtSize& dst) {
          struct v4l2_frmivalenum frmival;
          memset(&frmival, 0, sizeof(frmival));
          frmival.index = 0;
          frmival.pixel_format = fmt.pixelformat;
          frmival.width = static_cast<uint32_t>(w);
          frmival.height = static_cast<uint32_t>(h);

          if (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) != 0) {
            int saved_errno = errno;
            // 一部ドライバは ENUM_FRAMEINTERVALS を未実装のため、
            // VIDIOC_G_PARM から timeperframe を取得してフォールバック表示する
            std::cerr << "VIDIOC_ENUM_FRAMEINTERVALS failed: "
                      << strerror(saved_errno) << " (" << saved_errno
                      << ") for format="
                      << webrtc::GetFourccName(fmt.pixelformat) << ", size="
                      << w << "x" << h << ". Trying fallback via VIDIOC_G_PARM."
                      << std::endl;
            struct v4l2_format tryfmt;
            memset(&tryfmt, 0, sizeof(tryfmt));
            tryfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            tryfmt.fmt.pix.width = static_cast<uint32_t>(w);
            tryfmt.fmt.pix.height = static_cast<uint32_t>(h);
            tryfmt.fmt.pix.pixelformat = fmt.pixelformat;
            (void)ioctl(fd, VIDIOC_TRY_FMT, &tryfmt);  // 成否は問わない

            struct v4l2_streamparm parm;
            memset(&parm, 0, sizeof(parm));
            parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(fd, VIDIOC_G_PARM, &parm) == 0) {
              // capability に V4L2_CAP_TIMEPERFRAME が立っていれば有効
              if (parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
                V4L2FrameInterval iv;
                iv.index = 0;
                iv.type = V4L2FrameIntervalType::DISCRETE;
                iv.numerator = parm.parm.capture.timeperframe.numerator;
                iv.denominator = parm.parm.capture.timeperframe.denominator;
                if (iv.numerator > 0 && iv.denominator > 0) {
                  dst.intervals.push_back(iv);
                }
              } else {
                std::cerr << "VIDIOC_G_PARM returned without V4L2_CAP_TIMEPERFRAME"
                          << ", format="
                          << webrtc::GetFourccName(fmt.pixelformat)
                          << ", size=" << w << "x" << h << std::endl;
              }
            } else {
              int e2 = errno;
              std::cerr << "VIDIOC_G_PARM failed: " << strerror(e2) << " (" << e2
                        << ") for format="
                        << webrtc::GetFourccName(fmt.pixelformat)
                        << ", size=" << w << "x" << h << std::endl;
            }
            if (dst.intervals.empty()) {
              std::cerr << "Could not obtain frame intervals (fps) even with fallback"
                        << ", format=" << webrtc::GetFourccName(fmt.pixelformat)
                        << ", size=" << w << "x" << h << std::endl;
            }
            return;  // フォールバックの有無にかかわらず終了
          }

          if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            do {
              V4L2FrameInterval interval;
              interval.index = frmival.index;
              interval.type = V4L2FrameIntervalType::DISCRETE;
              interval.numerator = frmival.discrete.numerator;
              interval.denominator = frmival.discrete.denominator;
              dst.intervals.push_back(interval);
              frmival.index++;
            } while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0 &&
                     frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE);
          } else if (frmival.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
            V4L2FrameInterval interval;
            interval.index = 0;
            interval.type = V4L2FrameIntervalType::STEPWISE;
            interval.min_numerator = frmival.stepwise.min.numerator;
            interval.min_denominator = frmival.stepwise.min.denominator;
            interval.max_numerator = frmival.stepwise.max.numerator;
            interval.max_denominator = frmival.stepwise.max.denominator;
            interval.step_numerator = frmival.stepwise.step.numerator;
            interval.step_denominator = frmival.stepwise.step.denominator;
            dst.intervals.push_back(interval);
          } else if (frmival.type == V4L2_FRMIVAL_TYPE_CONTINUOUS) {
            V4L2FrameInterval interval;
            interval.index = 0;
            interval.type = V4L2FrameIntervalType::CONTINUOUS;
            interval.min_numerator = frmival.stepwise.min.numerator;
            interval.min_denominator = frmival.stepwise.min.denominator;
            interval.max_numerator = frmival.stepwise.max.numerator;
            interval.max_denominator = frmival.stepwise.max.denominator;
            interval.step_numerator = 1;
            interval.step_denominator = 1;
            dst.intervals.push_back(interval);
          }
        };

        // DISCRETE は 1 種のみ、範囲は min/mid/max の 3 種
        if (frame_size.type == V4L2FrameSizeType::DISCRETE) {
          V4L2IntervalsAtSize at;
          at.point = V4L2SizePoint::MIN;  // 意味的には唯一のサイズ
          at.width = frame_size.width;
          at.height = frame_size.height;
          enum_intervals(at.width, at.height, at);
          // 既存との互換のため intervals も埋める
          frame_size.intervals = at.intervals;
        } else {
          auto min_w = frame_size.min_width;
          auto max_w = frame_size.max_width;
          auto step_w = std::max(1, frame_size.step_width);
          auto min_h = frame_size.min_height;
          auto max_h = frame_size.max_height;
          auto step_h = std::max(1, frame_size.step_height);

          auto mid_w = min_w + ((max_w - min_w) / 2 / step_w) * step_w;
          auto mid_h = min_h + ((max_h - min_h) / 2 / step_h) * step_h;

          V4L2IntervalsAtSize at_min{V4L2SizePoint::MIN, min_w, min_h, {}};
          enum_intervals(at_min.width, at_min.height, at_min);
          frame_size.intervals_at_sizes.push_back(at_min);

          V4L2IntervalsAtSize at_mid{V4L2SizePoint::MID, mid_w, mid_h, {}};
          enum_intervals(at_mid.width, at_mid.height, at_mid);
          frame_size.intervals_at_sizes.push_back(at_mid);

          V4L2IntervalsAtSize at_max{V4L2SizePoint::MAX, max_w, max_h, {}};
          enum_intervals(at_max.width, at_max.height, at_max);
          frame_size.intervals_at_sizes.push_back(at_max);
        }

        format_desc.frame_sizes.push_back(frame_size);
        frmsize.index++;
      }
      device.format_descriptions.push_back(format_desc);
      fmt.index++;
    }

    // フォーマットが一つも列挙できなかったデバイスは除外する
    if (device.format_descriptions.size() == 0) {
      close(fd);
      continue;
    }

    enabled_devices.push_back(device);
    close(fd);
  }

  return enabled_devices;
}

std::string FormatV4L2Devices(const std::vector<V4L2Device>& devices) {
  std::string result;
  for (const auto& device : devices) {
    result += FormatV4L2Device(device, 2);
  }
  return result;
}
std::string FormatV4L2Device(const V4L2Device& device, int indent) {
  std::string indent_str(indent, ' ');
  std::stringstream ss;
  ss << indent_str << "[" << device.path << "] " << device.card << "" << " ("
     << device.bus_info << "):\n";
  ss << indent_str << "  Supported formats:\n";
  for (const auto& format_desc : device.format_descriptions) {
    ss << FormatV4L2FormatDescription(format_desc, indent + 4);
  }
  return ss.str();
}
std::string FormatV4L2FormatDescription(
    const V4L2FormatDescription& format_desc,
    int indent) {
  std::string indent_str(indent, ' ');
  std::stringstream ss;
  ss << indent_str << "[" << format_desc.index << "] "
     << webrtc::GetFourccName(format_desc.pixel_format) << " ("
     << format_desc.description << ")\n";
  for (const auto& frame_size : format_desc.frame_sizes) {
    ss << FormatV4L2FrameSize(frame_size, indent + 2);
  }
  return ss.str();
}
std::string FormatV4L2FrameSize(const V4L2FrameSize& frame_size, int indent) {
  std::string indent_str(indent, ' ');
  std::stringstream ss;
  if (frame_size.type == V4L2FrameSizeType::DISCRETE) {
    ss << indent_str << frame_size.width << "x" << frame_size.height;
  } else {
    // RANGE 表現
    auto step_w = frame_size.step_width;
    auto step_h = frame_size.step_height;
    ss << indent_str << "W[" << frame_size.min_width << ":" << frame_size.max_width
       << ":" << (step_w == 0 ? 1 : step_w) << "] x H[" << frame_size.min_height
       << ":" << frame_size.max_height << ":" << (step_h == 0 ? 1 : step_h)
       << "]";
  }

  if (!frame_size.intervals.empty()) {
    ss << " (";
    // 代表して 1 つ目の type を見て体裁を変える
    if (frame_size.intervals[0].type == V4L2FrameIntervalType::DISCRETE) {
      bool first = true;
      for (const auto& interval : frame_size.intervals) {
        if (!first) ss << ", ";
        first = false;
        if (interval.numerator == 1) {
          ss << interval.denominator << " fps";
        } else {
          ss << interval.denominator << "/" << interval.numerator << " fps";
        }
      }
    } else {
      const auto& iv = frame_size.intervals[0];
      // 区間表示: fps は denominator/numerator
      // 便宜的に min/max の分数を表示
      ss << "fps min=" << iv.max_denominator << "/" << iv.max_numerator
         << ", max=" << iv.min_denominator << "/" << iv.min_numerator;
      if (iv.type == V4L2FrameIntervalType::STEPWISE) {
        ss << ", step=" << iv.step_denominator << "/" << iv.step_numerator;
      }
    }
    ss << ")";
  }
  ss << "\n";

  // 範囲サイズは代表サイズごとのフレームレートも表示
  if (!frame_size.intervals_at_sizes.empty()) {
    for (const auto& at : frame_size.intervals_at_sizes) {
      ss << FormatV4L2IntervalsAtSize(at, indent + 2);
    }
  }

  return ss.str();
}

std::string FormatV4L2IntervalsAtSize(const V4L2IntervalsAtSize& at,
                                      int indent) {
  std::string indent_str(indent, ' ');
  std::stringstream ss;
  const char* name = at.point == V4L2SizePoint::MIN
                         ? "min"
                         : (at.point == V4L2SizePoint::MID ? "mid" : "max");
  ss << indent_str << "[" << name << "] " << at.width << "x" << at.height;
  if (!at.intervals.empty()) {
    ss << " (";
    if (at.intervals[0].type == V4L2FrameIntervalType::DISCRETE) {
      bool first = true;
      for (const auto& iv : at.intervals) {
        if (!first) ss << ", ";
        first = false;
        if (iv.numerator == 1) {
          ss << iv.denominator << " fps";
        } else {
          ss << iv.denominator << "/" << iv.numerator << " fps";
        }
      }
    } else {
      const auto& iv = at.intervals[0];
      ss << "fps min=" << iv.max_denominator << "/" << iv.max_numerator
         << ", max=" << iv.min_denominator << "/" << iv.min_numerator;
      if (iv.type == V4L2FrameIntervalType::STEPWISE) {
        ss << ", step=" << iv.step_denominator << "/" << iv.step_numerator;
      }
    }
    ss << ")";
  }
  ss << "\n";
  return ss.str();
}

}  // namespace sora
