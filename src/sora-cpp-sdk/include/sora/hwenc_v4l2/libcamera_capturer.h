#ifndef SORA_HWENC_V4L2_LIBCAMERA_CAPTURER_H_
#define SORA_HWENC_V4L2_LIBCAMERA_CAPTURER_H_

#include <memory>
#include <mutex>
#include <queue>

#include "sora/scalable_track_source.h"
#include "sora/v4l2/v4l2_video_capturer.h"

namespace sora {

struct LibcameraCapturerConfig : V4L2VideoCapturerConfig {
  LibcameraCapturerConfig() {}
  LibcameraCapturerConfig(const V4L2VideoCapturerConfig& config) {
    *static_cast<V4L2VideoCapturerConfig*>(this) = config;
  }
  LibcameraCapturerConfig(const LibcameraCapturerConfig& config) {
    *this = config;
  }
  // native_frame_output == true の場合、キャプチャしたデータを kNative なフレームとして渡す。
  // native_frame_output == false の場合、データをコピーして I420Buffer なフレームを作って渡す。
  // 前者の方が効率が良いけれども、kNative なフレームはサイマルキャスト時に自動で
  // リサイズしてくれないので、状況に応じて使い分けるのが良い。
  bool native_frame_output = false;
  // libcamera のコントロール設定。key value の形式。
  std::vector<std::pair<std::string, std::string>> controls;
};

// Raspberry Pi 専用のカメラからの映像を取得するクラス
// 出力の形式として、fd そのままで取得する形式と、メモリ上にコピーして取得する形式がある
// 渡されるフレームバッファは、fd そのままで取得する場合は V4L2NativeBuffer クラスになり、
// メモリ上にコピーする場合は webrtc::I420Buffer クラスになる。
class LibcameraCapturer : public ScalableVideoTrackSource {
 public:
  using ScalableVideoTrackSource::ScalableVideoTrackSource;
  static webrtc::scoped_refptr<LibcameraCapturer> Create(
      LibcameraCapturerConfig config);
};

}  // namespace sora

#endif  // LIBCAMERA_CAPTURER_H_