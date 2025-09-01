#include "sora/v4l2/v4l2_device.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <sstream>

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
        if (frmsize.type != V4L2_FRMSIZE_TYPE_DISCRETE) {
          frmsize.index++;
          continue;
        }
        V4L2DiscreteFrameSize frame_size;
        frame_size.index = frmsize.index;
        frame_size.width = frmsize.discrete.width;
        frame_size.height = frmsize.discrete.height;

        // 各フォーマットの各解像度がサポートしているフレームレートの列挙
        struct v4l2_frmivalenum frmival;
        memset(&frmival, 0, sizeof(frmival));
        frmival.index = 0;
        frmival.pixel_format = fmt.pixelformat;
        frmival.width = frmsize.discrete.width;
        frmival.height = frmsize.discrete.height;

        while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0) {
          if (frmival.type != V4L2_FRMIVAL_TYPE_DISCRETE) {
            frmival.index++;
            continue;
          }
          V4L2DiscreteInterval interval;
          interval.index = frmival.index;
          interval.numerator = frmival.discrete.numerator;
          interval.denominator = frmival.discrete.denominator;
          frame_size.intervals.push_back(interval);
          frmival.index++;
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
    ss << FormatV4L2DiscreteFrameSize(frame_size, indent + 2);
  }
  return ss.str();
}
std::string FormatV4L2DiscreteFrameSize(const V4L2DiscreteFrameSize& frame_size,
                                        int indent) {
  std::string indent_str(indent, ' ');
  std::stringstream ss;
  ss << indent_str << frame_size.width << "x" << frame_size.height;

  ss << " (";
  bool first = true;
  for (const auto& interval : frame_size.intervals) {
    if (!first) {
      ss << ", ";
    }
    first = false;
    if (interval.numerator == 1) {
      ss << interval.denominator << " fps";
    } else {
      ss << interval.denominator << "/" << interval.numerator << " fps";
    }
  }
  ss << ")\n";
  return ss.str();
}

}  // namespace sora