cmake_minimum_required(VERSION 3.16.2)

cmake_policy(SET CMP0091 NEW)

project(momo C CXX)

if (MSVC)
  enable_language(CUDA)

  # 文字コードを utf-8 として扱うのと、シンボルテーブル数を増やす
  #add_compile_options("/utf-8;/bigobj")
  #add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
  #add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
  #add_compile_options("$<$<C_COMPILER_ID:MSVC>:/bigobj>")
  #add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/bigobj>")

  # CRTライブラリを静的リンクさせる
  # CMake 3.15 以上なら CMAKE_MSVC_RUNTIME_LIBRARY が使える
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")

  set(_LINK_LIBRARIES
    dbghelp.lib
    delayimp.lib
    dnsapi.lib
    msimg32.lib
    oleaut32.lib
    psapi.lib
    shell32.lib
    shlwapi.lib
    usp10.lib
    version.lib
    wininet.lib
    winmm.lib
    ws2_32.lib
    amstrmid.lib
    Strmiids.lib
    crypt32.lib
    dmoguids.lib
    iphlpapi.lib
    msdmo.lib
    Secur32.lib
    wmcodecdspuuid.lib)

  set(_COMPILE_DEFINITIONS
    _CONSOLE
    _WIN32_WINNT=0x0A00
    WEBRTC_WIN
    NOMINMAX
    WIN32_LEAN_AND_MEAN
    UNICODE
    _UNICODE
    USE_NVCODEC_ENCODER=1)
elseif (APPLE)

  set(_LINK_LIBRARIES
    "-framework Foundation"
    "-framework AVFoundation"
    "-framework CoreServices"
    "-framework CoreFoundation"
    "-framework AudioUnit"
    "-framework AudioToolbox"
    "-framework CoreAudio"
    "-framework CoreGraphics"
    "-framework CoreMedia"
    "-framework CoreVideo"
    "-framework VideoToolbox"
    "-framework AppKit"
    "-framework Metal"
    "-framework MetalKit"
    "-framework OpenGL")

  set(_COMPILE_DEFINITIONS
    WEBRTC_POSIX
    WEBRTC_MAC)
endif ()

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME ON)

find_package(WebRTC REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(JSON REQUIRED)
find_package(CLI11 REQUIRED)
find_package(SDL2 REQUIRED)
find_package(NvCodec REQUIRED)

add_executable(momo)

set_target_properties(momo
  PROPERTIES
    COMPILE_OPTIONS "$<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler;/utf-8 /bigobj>$<$<COMPILE_LANGUAGE:CXX>:/utf-8;/bigobj>")

if (WIN32)
elseif (APPLE)
  set(_ADDITIONAL_SOURCES
    src/mac_helper/mac_capturer.mm
    src/mac_helper/macos_version.mm
    src/mac_helper/objc_codec_factory_helper.mm
  )

  # Objective-C++ の設定
  set_source_files_properties(${_ADDITIONAL_SOURCES}
    PROPERTIES
      COMPILE_OPTIONS "-x;objective-c++;-fmodules;-fcxx-modules")

  set_target_properties(momo PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif ()

target_sources(momo
  PRIVATE
    src/main.cpp
    src/momo_version.cpp
    src/util.cpp
    src/watchdog.cpp
    src/ayame/ayame_server.cpp
    src/ayame/ayame_websocket_client.cpp
    src/p2p/p2p_connection.cpp
    src/p2p/p2p_server.cpp
    src/p2p/p2p_session.cpp
    src/p2p/p2p_websocket_session.cpp
    src/hwenc_nvcodec/nvcodec_h264_encoder.cpp
    src/rtc/connection.cpp
    src/rtc/device_video_capturer.cpp
    src/rtc/h264_format.cpp
    src/rtc/hw_video_decoder_factory.cpp
    src/rtc/hw_video_encoder_factory.cpp
    src/rtc/manager.cpp
    src/rtc/native_buffer.cpp
    src/rtc/observer.cpp
    src/rtc/scalable_track_source.cpp
    src/sdl_renderer/sdl_renderer.cpp
    src/serial_data_channel/serial_data_channel.cpp
    src/serial_data_channel/serial_data_manager.cpp
    src/signal_listener.cpp
    src/sora/sora_server.cpp
    src/sora/sora_session.cpp
    src/sora/sora_websocket_client.cpp
    src/ws/websocket.cpp
    ${_ADDITIONAL_SOURCES})

# NVIDIA Video Codec SDK
target_sources(momo
  PRIVATE
    ${NVCODEC_ROOT_DIR}/Samples/NvCodec/NvEncoder/NvEncoder.cpp
    ${NVCODEC_ROOT_DIR}/Samples/NvCodec/NvEncoder/NvEncoderCuda.cpp)
target_include_directories(momo
  PRIVATE
    ${NVCODEC_ROOT_DIR}/Samples/NvCodec)
set_source_files_properties(${NVCODEC_ROOT_DIR}/Samples/NvCodec/NvEncoder/NvEncoderCuda.cpp
  PROPERTIES
    LANGUAGE CUDA
    CUDA_SEPARABLE_COMPILATION ON)

target_include_directories(momo PRIVATE src)

set_target_properties(momo PROPERTIES CXX_STANDARD 14 C_STANDARD 99)

target_link_libraries(momo
  WebRTC::WebRTC
  Boost::filesystem
  JSON::JSON
  CLI11::CLI11
  SDL2::SDL2-static
  SDL2::SDL2main
  NvCodec::nvencodeapi
  cuda.lib
  ${_LINK_LIBRARIES})

string(SUBSTRING ${MOMO_COMMIT} 0 8 MOMO_COMMIT_SHORT)
string(SUBSTRING ${WEBRTC_COMMIT} 0 8 WEBRTC_COMMIT_SHORT)

target_compile_definitions(momo PRIVATE
  USE_SDL2=1
  _THREAD_SAVE
  USE_H264=1
  MOMO_VERSION="${MOMO_VERSION}"
  MOMO_COMMIT_SHORT="${MOMO_COMMIT_SHORT}"
  WEBRTC_READABLE_VERSION="${WEBRTC_READABLE_VERSION}"
  WEBRTC_COMMIT_SHORT="${WEBRTC_COMMIT_SHORT}"
  WEBRTC_BUILD_VERSION="${WEBRTC_BUILD_VERSION}"
  ${_COMPILE_DEFINITIONS})