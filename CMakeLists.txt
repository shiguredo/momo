cmake_minimum_required(VERSION 3.16)

# Only interpret if() arguments as variables or keywords when unquoted.
cmake_policy(SET CMP0054 NEW)
# MSVC runtime library flags are selected by an abstraction.
cmake_policy(SET CMP0091 NEW)

set(TARGET_OS "" CACHE STRING "ビルド対象の動作する OS。\n有効な値は windows, macos, linux")
set(TARGET_OS_LINUX "" CACHE STRING "TARGET_OS が linux の場合の詳細な OS の情報。")
set(TARGET_ARCH "" CACHE STRING "ビルド対象の動作する CPU。\n有効な値は x86_64, arm")
set(TARGET_ARCH_ARM "" CACHE STRING "TARGET_ARCH が arm の場合の詳細な CPU の情報。\n有効な値は armv6, armv7, armv8")
set(USE_ROS OFF CACHE BOOL "ROS を使っているかどうか")
set(USE_MMAL_ENCODER OFF CACHE BOOL "MMAL ハードウェアエンコーダを利用するかどうか")
set(USE_JETSON_ENCODER OFF CACHE BOOL "Jetson のハードウェアエンコーダを利用するかどうか")
set(USE_H264 OFF CACHE BOOL "H264 を利用するかどうか")
set(USE_SDL2 OFF CACHE BOOL "SDL2 による画面出力を利用するかどうか")
set(BOOST_ROOT_DIR "" CACHE PATH "Boost のインストール先ディレクトリ")
set(SDL2_ROOT_DIR "" CACHE PATH "SDL2 のインストール先ディレクトリ")
set(JSON_ROOT_DIR "" CACHE PATH "nlohmann/json のインストール先ディレクトリ")
set(CLI11_ROOT_DIR "" CACHE PATH "CLI11 のインストール先ディレクトリ")
set(WEBRTC_INCLUDE_DIR "" CACHE PATH "WebRTC のインクルードディレクトリ")
set(WEBRTC_LIBRARY_DIR "" CACHE PATH "WebRTC のライブラリディレクトリ")
set(WEBRTC_LIBRARY_NAME "webrtc" CACHE STRING "WebRTC のライブラリ名")
set(CLANG_ROOT "" CACHE PATH "コンパイルに利用する clang コンパイラの配置場所")
set(USE_LIBCXX OFF CACHE BOOL "libstdc++ の代わりに libc++ を使うかどうか")
set(LIBCXX_INCLUDE_DIR "" CACHE PATH "libc++ を使う場合の libc++ のインクルードディレクトリ\n空文字だった場合はデフォルト検索パスの libc++ を利用する")
set(SYSROOT "" CACHE PATH "TARGET_ARCH が arm の場合のクロスビルド用の RootFS ディレクトリ")
set(MOMO_VERSION "unspecified" CACHE STRING "Momo のバージョン")
set(MOMO_COMMIT "<commit>" CACHE STRING "Momo のコミットハッシュ")
set(WEBRTC_BUILD_VERSION "unspecified" CACHE STRING "webrtc-build のバージョン")
set(WEBRTC_READABLE_VERSION "unspecified" CACHE STRING "WebRTC の読みやすいバージョン")
set(WEBRTC_COMMIT "<commit>" CACHE STRING "WebRTC のコミットハッシュ")

if(MOMO_PACKAGE_NAME STREQUAL "windows")
  set(_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/windows/_install)

  set(TARGET_OS "windows")
  set(TARGET_ARCH "x86_64")
  set(USE_SDL2 ON)
  set(BOOST_ROOT_DIR ${_INSTALL_DIR}/boost)
  set(SDL2_ROOT_DIR ${_INSTALL_DIR}/SDL2)
  set(JSON_ROOT_DIR ${_INSTALL_DIR}/json)
  set(CLI11_ROOT_DIR ${_INSTALL_DIR}/CLI11)
  set(WEBRTC_INCLUDE_DIR ${_INSTALL_DIR}/webrtc/include)
  set(WEBRTC_LIBRARY_DIR ${_INSTALL_DIR}/webrtc/release)

  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${SDL2_ROOT_DIR}/cmake)
  list(APPEND CMAKE_PREFIX_PATH ${BOOST_ROOT_DIR} ${SDL2_ROOT_DIR})
elseif(MOMO_PACKAGE_NAME STREQUAL "macos")
endif()

project(momo C CXX OBJCXX)

list(APPEND MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME ON)

find_package(WebRTC REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(JSON REQUIRED)
find_package(CLI11 REQUIRED)
find_package(SDL2 REQUIRED)

add_executable(momo)

string(SUBSTRING ${MOMO_COMMIT} 0 8 MOMO_COMMIT_SHORT)
string(SUBSTRING ${WEBRTC_COMMIT} 0 8 WEBRTC_COMMIT_SHORT)
configure_file(src/momo_version.gen.h.template ${CMAKE_CURRENT_BINARY_DIR}/momo_version.gen.h)
target_include_directories(momo PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

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
)

target_include_directories(momo PRIVATE src)

set_target_properties(momo PROPERTIES CXX_STANDARD 14 C_STANDARD 99)

target_compile_definitions(momo
  PRIVATE
    USE_SDL2=1
    _THREAD_SAVE
    USE_H264=1
)

target_link_libraries(momo
  PRIVATE
    WebRTC::WebRTC
    Boost::filesystem
    JSON::JSON
    CLI11::CLI11
)
if (USE_SDL2)
  target_link_libraries(momo
    PRIVATE
      SDL2::SDL2-static
      SDL2::SDL2main
  )
endif()

if (TARGET_OS STREQUAL "windows")
  set_target_properties(momo
    PROPERTIES
      # 文字コードを utf-8 として扱うのと、シンボルテーブル数を増やす
      COMPILE_OPTIONS "/utf-8;/bigobj"
      # CRTライブラリを静的リンクさせる
      # CMake 3.15 以上なら MSVC_RUNTIME_LIBRARY が使える
      MSVC_RUNTIME_LIBRARY "MultiThreaded"
  )

  target_link_libraries(momo
    PRIVATE
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
      wmcodecdspuuid.lib
  )

  target_compile_definitions(momo
    PRIVATE
      _CONSOLE
      _WIN32_WINNT=0x0A00
      WEBRTC_WIN
      NOMINMAX
      WIN32_LEAN_AND_MEAN
      UNICODE
      _UNICODE
  )
elseif (TARGET_OS STREQUAL "macos")
  target_sources(momo
    PRIVATE
      src/mac_helper/mac_capturer.mm
      src/mac_helper/macos_version.mm
      src/mac_helper/objc_codec_factory_helper.mm
  )

  set_target_properties(momo PROPERTIES CXX_VISIBILITY_PRESET hidden)

  target_link_libraries(momo
    PRIVATE
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
      "-framework OpenGL"
  )

  target_compile_definitions(momo
    PRIVATE
      WEBRTC_POSIX
      WEBRTC_MAC
  )
endif ()
