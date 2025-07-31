#include "libcamerac.h"

#include <libcamera/base/span.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/property_ids.h>

#include <string.h>
#include <algorithm>
#include <sstream>
#include <vector>

extern "C" {

// libcamerac_CameraManager

libcamerac_CameraManager* libcamerac_CameraManager_new() {
  return (libcamerac_CameraManager*)new libcamera::CameraManager();
}
void libcamerac_CameraManager_destroy(libcamerac_CameraManager* p) {
  delete (libcamera::CameraManager*)p;
}
int libcamerac_CameraManager_start(libcamerac_CameraManager* p) {
  return ((libcamera::CameraManager*)p)->start();
}
void libcamerac_CameraManager_stop(libcamerac_CameraManager* p) {
  ((libcamera::CameraManager*)p)->stop();
}
libcamerac_vector_Camera* libcamerac_CameraManager_cameras(
    libcamerac_CameraManager* p) {
  return (libcamerac_vector_Camera*)new std::vector<
      std::shared_ptr<libcamera::Camera>>(
      ((libcamera::CameraManager*)p)->cameras());
}
libcamerac_Camera* libcamerac_CameraManager_get(libcamerac_CameraManager* p,
                                                const char* id) {
  return (libcamerac_Camera*)new std::shared_ptr<libcamera::Camera>(
      ((libcamera::CameraManager*)p)->get(id));
}

// libcamerac_vector_Camera

void libcamerac_vector_Camera_destroy(libcamerac_vector_Camera* p) {
  delete (std::vector<std::shared_ptr<libcamera::Camera>>*)p;
}
int libcamerac_vector_Camera_size(libcamerac_vector_Camera* p) {
  return ((std::vector<std::shared_ptr<libcamera::Camera>>*)p)->size();
}
libcamerac_Camera* libcamerac_vector_Camera_at(libcamerac_vector_Camera* p,
                                               int i) {
  return (libcamerac_Camera*)&(
             (std::vector<std::shared_ptr<libcamera::Camera>>*)p)
      ->at(i);
}

// libcamerac_vector_StreamRole

libcamerac_vector_StreamRole* libcamerac_vector_StreamRole_new() {
  return (
      libcamerac_vector_StreamRole*)new std::vector<libcamera::StreamRole>();
}
void libcamerac_vector_StreamRole_destroy(libcamerac_vector_StreamRole* p) {
  delete (std::vector<libcamera::StreamRole>*)p;
}
void libcamerac_vector_StreamRole_push_back(libcamerac_vector_StreamRole* p,
                                            libcamerac_StreamRole v) {
  ((std::vector<libcamera::StreamRole>*)p)->push_back((libcamera::StreamRole)v);
}
int libcamerac_vector_StreamRole_size(libcamerac_vector_StreamRole* p) {
  return ((std::vector<libcamera::StreamRole>*)p)->size();
}
void libcamerac_vector_StreamRole_clear(libcamerac_vector_StreamRole* p) {
  ((std::vector<libcamera::StreamRole>*)p)->clear();
}

// libcamerac_Camera

libcamerac_Camera* libcamerac_Camera_own(libcamerac_Camera* p) {
  return (libcamerac_Camera*)new std::shared_ptr<libcamera::Camera>(
      *(std::shared_ptr<libcamera::Camera>*)p);
}
void libcamerac_Camera_disown(libcamerac_Camera* p) {
  delete (std::shared_ptr<libcamera::Camera>*)p;
}
const char* libcamerac_Camera_id(libcamerac_Camera* p) {
  return (*(std::shared_ptr<libcamera::Camera>*)p)->id().c_str();
}
libcamerac_CameraConfiguration* libcamerac_Camera_generateConfiguration(
    libcamerac_Camera* p,
    libcamerac_vector_StreamRole* roles) {
  return (libcamerac_CameraConfiguration*)(*(std::shared_ptr<
                                               libcamera::Camera>*)p)
      ->generateConfiguration(*(std::vector<libcamera::StreamRole>*)roles)
      .release();
}
int libcamerac_Camera_configure(libcamerac_Camera* p,
                                libcamerac_CameraConfiguration* config) {
  return (*(std::shared_ptr<libcamera::Camera>*)p)
      ->configure((libcamera::CameraConfiguration*)config);
}
libcamerac_Request* libcamerac_Camera_createRequest(libcamerac_Camera* p) {
  return (libcamerac_Request*)(*(std::shared_ptr<libcamera::Camera>*)p)
      ->createRequest()
      .release();
}
int libcamerac_Camera_queueRequest(libcamerac_Camera* p,
                                   libcamerac_Request* request) {
  return (*(std::shared_ptr<libcamera::Camera>*)p)
      ->queueRequest((libcamera::Request*)request);
}
int libcamerac_Camera_start(libcamerac_Camera* p,
                            const libcamerac_ControlList* control_list) {
  return (*(std::shared_ptr<libcamera::Camera>*)p)
      ->start((const libcamera::ControlList*)control_list);
}
int libcamerac_Camera_stop(libcamerac_Camera* p) {
  return (*(std::shared_ptr<libcamera::Camera>*)p)->stop();
}
int libcamerac_Camera_acquire(libcamerac_Camera* p) {
  return (*(std::shared_ptr<libcamera::Camera>*)p)->acquire();
}
int libcamerac_Camera_release(libcamerac_Camera* p) {
  return (*(std::shared_ptr<libcamera::Camera>*)p)->release();
}
libcamerac_Signal_Request* libcamerac_Camera_requestCompleted(
    libcamerac_Camera* p) {
  return (libcamerac_Signal_Request*)&(*(std::shared_ptr<libcamera::Camera>*)p)
      ->requestCompleted;
}

// libcamerac_CameraConfiguration

void libcamerac_CameraConfiguration_destroy(libcamerac_CameraConfiguration* p) {
  delete (libcamera::CameraConfiguration*)p;
}
libcamerac_StreamConfiguration* libcamerac_CameraConfiguration_at(
    libcamerac_CameraConfiguration* p,
    int i) {
  return (libcamerac_StreamConfiguration*)&((libcamera::CameraConfiguration*)p)
      ->at(i);
}
libcamerac_CameraConfiguration_Status libcamerac_CameraConfiguration_validate(
    libcamerac_CameraConfiguration* p) {
  return (libcamerac_CameraConfiguration_Status)((libcamera::
                                                      CameraConfiguration*)p)
      ->validate();
}

// libcamerac_StreamConfiguration

libcamerac_Stream* libcamerac_StreamConfiguration_stream(
    libcamerac_StreamConfiguration* p) {
  return (libcamerac_Stream*)((libcamera::StreamConfiguration*)p)->stream();
}
int libcamerac_StreamConfiguration_get_size_width(
    libcamerac_StreamConfiguration* p) {
  return ((libcamera::StreamConfiguration*)p)->size.width;
}
int libcamerac_StreamConfiguration_get_size_height(
    libcamerac_StreamConfiguration* p) {
  return ((libcamera::StreamConfiguration*)p)->size.height;
}
int libcamerac_StreamConfiguration_get_stride(
    libcamerac_StreamConfiguration* p) {
  return ((libcamera::StreamConfiguration*)p)->stride;
}
void libcamerac_StreamConfiguration_set_pixelFormat(
    libcamerac_StreamConfiguration* p,
    libcamerac_PixelFormat* pixelFormat) {
  ((libcamera::StreamConfiguration*)p)->pixelFormat =
      *(libcamera::PixelFormat*)pixelFormat;
}
void libcamerac_StreamConfiguration_set_size_width(
    libcamerac_StreamConfiguration* p,
    int width) {
  ((libcamera::StreamConfiguration*)p)->size.width = width;
}
void libcamerac_StreamConfiguration_set_size_height(
    libcamerac_StreamConfiguration* p,
    int height) {
  ((libcamera::StreamConfiguration*)p)->size.height = height;
}
void libcamerac_StreamConfiguration_set_stride(
    libcamerac_StreamConfiguration* p,
    int stride) {
  ((libcamera::StreamConfiguration*)p)->stride = stride;
}
void libcamerac_StreamConfiguration_set_frameSize(
    libcamerac_StreamConfiguration* p,
    int frameSize) {
  ((libcamera::StreamConfiguration*)p)->frameSize = frameSize;
}
void libcamerac_StreamConfiguration_set_bufferCount(
    libcamerac_StreamConfiguration* p,
    int bufferCount) {
  ((libcamera::StreamConfiguration*)p)->bufferCount = bufferCount;
}
void libcamerac_StreamConfiguration_set_colorSpace(
    libcamerac_StreamConfiguration* p,
    libcamerac_ColorSpace* colorSpace) {
  ((libcamera::StreamConfiguration*)p)->colorSpace =
      *(libcamera::ColorSpace*)colorSpace;
}

// libcamerac_FrameBufferAllocator

libcamerac_FrameBufferAllocator* libcamerac_FrameBufferAllocator_new(
    libcamerac_Camera* camera) {
  return (libcamerac_FrameBufferAllocator*)new libcamera::FrameBufferAllocator(
      *(std::shared_ptr<libcamera::Camera>*)camera);
}
void libcamerac_FrameBufferAllocator_destroy(
    libcamerac_FrameBufferAllocator* p) {
  delete (libcamera::FrameBufferAllocator*)p;
}
int libcamerac_FrameBufferAllocator_allocate(libcamerac_FrameBufferAllocator* p,
                                             libcamerac_Stream* stream) {
  return ((libcamera::FrameBufferAllocator*)p)
      ->allocate((libcamera::Stream*)stream);
}
int libcamerac_FrameBufferAllocator_free(libcamerac_FrameBufferAllocator* p,
                                         libcamerac_Stream* stream) {
  return ((libcamera::FrameBufferAllocator*)p)
      ->free((libcamera::Stream*)stream);
}
libcamerac_FrameBuffer* libcamerac_FrameBufferAllocator_buffers_at(
    const libcamerac_FrameBufferAllocator* p,
    libcamerac_Stream* stream,
    int i) {
  return (libcamerac_FrameBuffer*)((const libcamera::FrameBufferAllocator*)p)
      ->buffers((libcamera::Stream*)stream)[i]
      .get();
}
int libcamerac_FrameBufferAllocator_buffers_size(
    const libcamerac_FrameBufferAllocator* p,
    libcamerac_Stream* stream) {
  return ((libcamera::FrameBufferAllocator*)p)
      ->buffers((libcamera::Stream*)stream)
      .size();
}

// libcamerac_FrameBuffer

int libcamerac_FrameBuffer_planes_size(const libcamerac_FrameBuffer* p) {
  return ((const libcamera::FrameBuffer*)p)->planes().size();
}
const libcamerac_FrameBuffer_Plane* libcamerac_FrameBuffer_planes_at(
    const libcamerac_FrameBuffer* p,
    int i) {
  return (const libcamerac_FrameBuffer_Plane*)&(
             (const libcamera::FrameBuffer*)p)
      ->planes()
      .at(i);
}

// libcamerac_FrameBuffer_Plane

int libcamerac_FrameBuffer_Plane_fd(const libcamerac_FrameBuffer_Plane* p) {
  return ((const libcamera::FrameBuffer::Plane*)p)->fd.get();
}
int libcamerac_FrameBuffer_Plane_length(const libcamerac_FrameBuffer_Plane* p) {
  return ((const libcamera::FrameBuffer::Plane*)p)->length;
}

// libcamerac_ControlList

libcamerac_ControlList* libcamerac_ControlList_controls() {
  return (libcamerac_ControlList*)new libcamera::ControlList(
      libcamera::controls::controls);
}
void libcamerac_ControlList_destroy(libcamerac_ControlList* p) {
  delete (libcamera::ControlList*)p;
}
void libcamerac_ControlList_clear(libcamerac_ControlList* p) {
  ((libcamera::ControlList*)p)->clear();
}
void libcamerac_ControlList_copy(const libcamerac_ControlList* p,
                                 libcamerac_ControlList* control_list) {
  *((libcamera::ControlList*)control_list) = *(const libcamera::ControlList*)p;
}

// 文字列をカンマで分割するヘルパー関数
static std::vector<std::string> split_string(const std::string& str, char delimiter) {
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

// enum コントロールの文字列値を数値に変換
static bool parse_enum_control(const std::string& name, const std::string& value_str, int32_t& result) {
  // 主要な enum コントロールのマッピング
  if (name == "AfMode") {
    if (value_str == "Manual" || value_str == "0") result = 0;
    else if (value_str == "Auto" || value_str == "1") result = 1;
    else if (value_str == "Continuous" || value_str == "2") result = 2;
    else return false;
  } else if (name == "AfRange") {
    if (value_str == "Normal" || value_str == "0") result = 0;
    else if (value_str == "Macro" || value_str == "1") result = 1;
    else if (value_str == "Full" || value_str == "2") result = 2;
    else return false;
  } else if (name == "AfSpeed") {
    if (value_str == "Normal" || value_str == "0") result = 0;
    else if (value_str == "Fast" || value_str == "1") result = 1;
    else return false;
  } else if (name == "AeMeteringMode") {
    if (value_str == "CentreWeighted" || value_str == "0") result = 0;
    else if (value_str == "Spot" || value_str == "1") result = 1;
    else if (value_str == "Matrix" || value_str == "2") result = 2;
    else return false;
  } else if (name == "AwbMode") {
    if (value_str == "Auto" || value_str == "0") result = 0;
    else if (value_str == "Incandescent" || value_str == "1") result = 1;
    else if (value_str == "Tungsten" || value_str == "2") result = 2;
    else if (value_str == "Fluorescent" || value_str == "3") result = 3;
    else if (value_str == "Indoor" || value_str == "4") result = 4;
    else if (value_str == "Daylight" || value_str == "5") result = 5;
    else if (value_str == "Cloudy" || value_str == "6") result = 6;
    else return false;
  } else if (name == "ExposureTimeMode" || name == "AnalogueGainMode") {
    if (value_str == "Auto" || value_str == "0") result = 0;
    else if (value_str == "Manual" || value_str == "1") result = 1;
    else return false;
  } else {
    // その他の enum は数値として解析
    result = std::stoi(value_str);
  }
  return true;
}

int libcamerac_ControlList_set_by_name(libcamerac_Camera* camera,
                                       libcamerac_ControlList* p,
                                       const char* name,
                                       const char* value_str) {
  auto cam = *(std::shared_ptr<libcamera::Camera>*)camera;
  auto list = (libcamera::ControlList*)p;
  
  // カメラのコントロールから名前で検索
  const libcamera::ControlId* controlId = nullptr;
  for (const auto& [control, info] : cam->controls()) {
    if (control->name() == name) {
      controlId = control;
      break;
    }
  }
  
  if (!controlId) {
    return -1;  // コントロールが見つからない
  }
  
  // 値の解析と設定
  try {
    libcamera::ControlValue value;
    std::string value_string(value_str);
    
    // 配列型かどうかチェック
    bool is_array = controlId->isArray();
    
    switch (controlId->type()) {
      case libcamera::ControlTypeBool: {
        bool val = (value_string == "1" || value_string == "true");
        value.set<bool>(val);
        break;
      }
      case libcamera::ControlTypeInteger32: {
        if (is_array) {
          // int32配列の場合（例：FrameDurationLimits）
          auto tokens = split_string(value_string, ',');
          std::vector<int32_t> values;
          for (const auto& token : tokens) {
            values.push_back(std::stoi(token));
          }
          value.set(libcamera::Span<const int32_t>(values.data(), values.size()));
        } else {
          // enum かもしれないので、まず文字列として解析を試みる
          int32_t val;
          if (!parse_enum_control(name, value_string, val)) {
            val = std::stoi(value_string);
          }
          value.set<int32_t>(val);
        }
        break;
      }
      case libcamera::ControlTypeInteger64: {
        if (is_array) {
          // int64配列の場合（例：FrameDurationLimits）
          auto tokens = split_string(value_string, ',');
          std::vector<int64_t> values;
          for (const auto& token : tokens) {
            values.push_back(std::stoll(token));
          }
          value.set(libcamera::Span<const int64_t>(values.data(), values.size()));
        } else {
          int64_t val = std::stoll(value_string);
          value.set<int64_t>(val);
        }
        break;
      }
      case libcamera::ControlTypeFloat: {
        if (is_array) {
          // float配列の場合（例：ColourGains）
          auto tokens = split_string(value_string, ',');
          std::vector<float> values;
          for (const auto& token : tokens) {
            values.push_back(std::stof(token));
          }
          value.set(libcamera::Span<const float>(values.data(), values.size()));
        } else {
          float val = std::stof(value_string);
          value.set<float>(val);
        }
        break;
      }
      case libcamera::ControlTypeRectangle: {
        // Rectangle型の場合（例：ScalerCrop）
        // フォーマット: x,y,width,height
        auto tokens = split_string(value_string, ',');
        if (tokens.size() != 4) {
          return -3;  // 不正なフォーマット
        }
        libcamera::Rectangle rect(
          std::stoi(tokens[0]),  // x
          std::stoi(tokens[1]),  // y
          std::stoi(tokens[2]),  // width
          std::stoi(tokens[3])   // height
        );
        if (is_array) {
          // Rectangle配列の場合（例：AfWindows）
          // セミコロンで複数のRectangleを区切る
          auto rect_strings = split_string(value_string, ';');
          std::vector<libcamera::Rectangle> rectangles;
          for (const auto& rect_str : rect_strings) {
            auto rect_tokens = split_string(rect_str, ',');
            if (rect_tokens.size() != 4) {
              return -3;
            }
            rectangles.emplace_back(
              std::stoi(rect_tokens[0]),
              std::stoi(rect_tokens[1]),
              std::stoi(rect_tokens[2]),
              std::stoi(rect_tokens[3])
            );
          }
          value.set(libcamera::Span<const libcamera::Rectangle>(rectangles.data(), rectangles.size()));
        } else {
          value.set(rect);
        }
        break;
      }
      default:
        return -2;  // サポートされていない型
    }
    
    list->set(controlId->id(), value);
    return 0;
  } catch (...) {
    return -3;  // 値の解析エラー
  }
}

// libcamerac_Request

void libcamerac_Request_destroy(libcamerac_Request* p) {
  delete (libcamera::Request*)p;
}
libcamerac_Request_Status libcamerac_Request_status(
    const libcamerac_Request* p) {
  return (libcamerac_Request_Status)((const libcamera::Request*)p)->status();
}
libcamerac_FrameBuffer* libcamerac_Request_findBuffer(
    const libcamerac_Request* p,
    const libcamerac_Stream* stream) {
  return (libcamerac_FrameBuffer*)((const libcamera::Request*)p)
      ->findBuffer((const libcamera::Stream*)stream);
}
int libcamerac_Request_addBuffer(libcamerac_Request* p,
                                 const libcamerac_Stream* stream,
                                 libcamerac_FrameBuffer* buffer) {
  return ((libcamera::Request*)p)
      ->addBuffer((const libcamera::Stream*)stream,
                  (libcamera::FrameBuffer*)buffer);
}
void libcamerac_Request_reuse(libcamerac_Request* p) {
  ((libcamera::Request*)p)->reuse();
}
const libcamerac_Request_BufferMap* libcamerac_Request_buffers(
    const libcamerac_Request* p) {
  return (const libcamerac_Request_BufferMap*)&((const libcamera::Request*)p)
      ->buffers();
}
libcamerac_ControlList* libcamerac_Request_controls(libcamerac_Request* p) {
  return (libcamerac_ControlList*)&((libcamera::Request*)p)->controls();
}

// libcamerac_Request_BufferMap

void libcamerac_Request_BufferMap_foreach(
    const libcamerac_Request_BufferMap* p,
    void (*callback)(const libcamerac_Stream* stream,
                     libcamerac_FrameBuffer* buffer,
                     void* user_data),
    void* user_data) {
  for (const auto& v : *(const libcamera::Request::BufferMap*)p) {
    callback((const libcamerac_Stream*)v.first,
             (libcamerac_FrameBuffer*)v.second, user_data);
  }
}

// libcamerac_PixelFormat

libcamerac_PixelFormat* libcamerac_PixelFormat_YUV420() {
  return (libcamerac_PixelFormat*)&libcamera::formats::YUV420;
}

// libcamerac_ColorSpace

//extern libcamerac_ColorSpace* libcamerac_ColorSpace_Jpeg() {
//  return (libcamerac_ColorSpace*)&libcamera::ColorSpace::Jpeg;
//}

// libcamerac_Signal_Request

struct Signal_Request {
  void (*callback)(libcamerac_Request* request, void* user_data);
  void* user_data;
  void Run(libcamera::Request* request) {
    callback((libcamerac_Request*)request, user_data);
  }
};
std::vector<std::shared_ptr<Signal_Request>> g_signal_requests;

void libcamerac_Signal_Request_connect(
    libcamerac_Signal_Request* p,
    void (*callback)(libcamerac_Request* request, void* user_data),
    void* user_data) {
  auto signal = std::make_shared<Signal_Request>();
  signal->callback = callback;
  signal->user_data = user_data;
  g_signal_requests.push_back(signal);
  ((libcamera::Signal<libcamera::Request*>*)p)
      ->connect(signal.get(), &Signal_Request::Run);
}
void libcamerac_Signal_Request_disconnect(
    libcamerac_Signal_Request* p,
    void (*callback)(libcamerac_Request* request, void* user_data),
    void* user_data) {
  auto it = std::find_if(g_signal_requests.begin(), g_signal_requests.end(),
                         [=](const std::shared_ptr<Signal_Request>& signal) {
                           return signal->callback == callback &&
                                  signal->user_data == user_data;
                         });
  if (it != g_signal_requests.end()) {
    ((libcamera::Signal<libcamera::Request*>*)p)
        ->disconnect(it->get(), &Signal_Request::Run);
    g_signal_requests.erase(it);
  };
}
}
