cmake_minimum_required(VERSION 3.16)

# Only interpret if() arguments as variables or keywords when unquoted.
cmake_policy(SET CMP0054 NEW)
# MSVC runtime library flags are selected by an abstraction.
cmake_policy(SET CMP0091 NEW)

set(MOMO_PACKAGE_NAME "" CACHE STRING "ビルド設定を自動化するパッケージ名")
set(TARGET_OS "" CACHE STRING "ビルド対象の動作する OS。\n有効な値は windows, macos, linux")
set(TARGET_OS_LINUX "" CACHE STRING "TARGET_OS が linux の場合の詳細な OS の情報。")
set(TARGET_ARCH "" CACHE STRING "ビルド対象の動作する CPU。\n有効な値は x86_64, arm")
set(TARGET_ARCH_ARM "" CACHE STRING "TARGET_ARCH が arm の場合の詳細な CPU の情報。\n有効な値は armv6, armv7, armv8")
set(USE_ROS OFF CACHE BOOL "ROS を使っているかどうか")
set(USE_NVCODEC_ENCODER OFF CACHE BOOL "NVIDIA VIDEO CODEC SDK のハードウェアエンコーダを利用するかどうか")
set(USE_MMAL_ENCODER OFF CACHE BOOL "MMAL ハードウェアエンコーダを利用するかどうか")
set(USE_JETSON_ENCODER OFF CACHE BOOL "Jetson のハードウェアエンコーダを利用するかどうか")
set(USE_H264 OFF CACHE BOOL "H264 を利用するかどうか")
set(USE_SDL2 OFF CACHE BOOL "SDL2 による画面出力を利用するかどうか")
set(BOOST_ROOT_DIR "" CACHE PATH "Boost のインストール先ディレクトリ\n空文字だった場合はデフォルト検索パスの Boost を利用する")
set(SDL2_ROOT_DIR "" CACHE PATH "SDL2 のインストール先ディレクトリ\n空文字だった場合はデフォルト検索パスの SDL2 を利用する")
set(JSON_ROOT_DIR "" CACHE PATH "nlohmann/json のインストール先ディレクトリ")
set(CLI11_ROOT_DIR "" CACHE PATH "CLI11 のインストール先ディレクトリ")
set(NVCODEC_ROOT_DIR "" CACHE PATH "NVIDIA VIDEO CODEC SDK のインストール先ディレクトリ")
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
  set(USE_NVCODEC_ENCODER ON)
  set(USE_H264 ON)
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

  set(_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/macos/_install)

  set(TARGET_OS "macos")
  set(TARGET_ARCH "x86_64")
  set(USE_H264 ON)
  set(USE_SDL2 ON)
  set(BOOST_ROOT_DIR ${_INSTALL_DIR}/boost)
  set(SDL2_ROOT_DIR ${_INSTALL_DIR}/SDL2)
  set(JSON_ROOT_DIR ${_INSTALL_DIR}/json)
  set(CLI11_ROOT_DIR ${_INSTALL_DIR}/CLI11)
  set(WEBRTC_INCLUDE_DIR ${_INSTALL_DIR}/webrtc/include)
  set(WEBRTC_LIBRARY_DIR ${_INSTALL_DIR}/webrtc/lib)
  set(CLANG_ROOT ${_INSTALL_DIR}/llvm/clang)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR ${_INSTALL_DIR}/llvm/libcxx/include)

elseif(MOMO_PACKAGE_NAME STREQUAL "raspbian-buster_armv6")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "raspbian-buster")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv6")
  set(USE_MMAL_ENCODER ON)
  set(USE_H264 ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(SDL2_ROOT_DIR /root/SDL2)
  set(JSON_ROOT_DIR /root/json)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)
  set(CLANG_ROOT /root/llvm/clang)
  set(SYSROOT /root/rootfs)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

  set(Boost_ARCHITECTURE 32)

elseif(MOMO_PACKAGE_NAME STREQUAL "raspbian-buster_armv7")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "raspbian-buster")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv7")
  set(USE_MMAL_ENCODER ON)
  set(USE_H264 ON)
  set(USE_SDL2 ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(SDL2_ROOT_DIR "")
  set(JSON_ROOT_DIR /root/json)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)
  set(CLANG_ROOT /root/llvm/clang)
  set(SYSROOT /root/rootfs)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

  set(Boost_ARCHITECTURE 32)

  list(APPEND CMAKE_PREFIX_PATH /root/rootfs/usr/lib/arm-linux-gnueabihf/cmake/SDL2)

elseif(MOMO_PACKAGE_NAME STREQUAL "ubuntu-16.04_armv7_ros")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-16.04")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv7")
  set(USE_ROS ON)
  set(USE_MMAL_ENCODER ON)
  set(USE_H264 ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(JSON_ROOT_DIR /root/json)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)
  set(WEBRTC_LIBRARY_NAME "webrtc_nolibcxx")
  set(CLANG_ROOT /root/llvm/clang)
  set(SYSROOT /root/rootfs)

  set(Boost_ARCHITECTURE 32)

elseif(MOMO_PACKAGE_NAME STREQUAL "ubuntu-18.04_armv8")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-18.04")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv8")
  set(BOOST_ROOT_DIR /root/boost)
  set(JSON_ROOT_DIR /root/json)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)
  set(CLANG_ROOT /root/llvm/clang)
  set(SYSROOT /root/rootfs)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

elseif(MOMO_PACKAGE_NAME STREQUAL "ubuntu-18.04_armv8_jetson_nano")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-18.04")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv8")
  set(USE_JETSON_ENCODER ON)
  set(USE_H264 ON)
  set(USE_SDL2 ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(JSON_ROOT_DIR /root/json)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(SDL2_ROOT_DIR /root/SDL2)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)
  set(CLANG_ROOT /root/llvm/clang)
  set(SYSROOT /root/rootfs)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

elseif(MOMO_PACKAGE_NAME STREQUAL "ubuntu-16.04_x86_64_ros")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-16.04")
  set(TARGET_ARCH "x86_64")
  set(USE_ROS ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(JSON_ROOT_DIR /root/json)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)
  set(WEBRTC_LIBRARY_NAME "webrtc_nolibcxx")
  set(CLANG_ROOT /root/llvm/clang)

elseif(MOMO_PACKAGE_NAME STREQUAL "ubuntu-18.04_x86_64")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-18.04")
  set(TARGET_ARCH "x86_64")
  set(USE_SDL2 ON)
  set(USE_NVCODEC_ENCODER ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(JSON_ROOT_DIR /root/json)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(SDL2_ROOT_DIR /root/SDL2)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)

  # /root/llvm/clang にあるコンパイラは使わず、apt でインストールした clang-8 を利用する
  set(CMAKE_C_COMPILER clang-8)
  set(CMAKE_CXX_COMPILER clang++-8)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

else()
endif()

if (USE_ROS)
  if (TARGET_OS_LINUX STREQUAL "ubuntu-16.04")
    set(ROS_VERSION "kinetic")
  elseif (TARGET_OS_LINUX STREQUAL "ubuntu-18.04")
    set(ROS_VERSION "melodic")
  else()
    message(FATAL_ERROR "ROS_VERSION が決定できません")
  endif()
endif()


if (NOT "${CLANG_ROOT}" STREQUAL "")
  set(CMAKE_C_COMPILER ${CLANG_ROOT}/bin/clang)
  set(CMAKE_CXX_COMPILER ${CLANG_ROOT}/bin/clang++)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if (NOT "${BOOST_ROOT_DIR}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH ${BOOST_ROOT_DIR})
endif()

if (USE_SDL2 AND (NOT "${SDL2_ROOT_DIR}" STREQUAL ""))
  list(APPEND CMAKE_MODULE_PATH ${SDL2_ROOT_DIR}/cmake)
  list(APPEND CMAKE_PREFIX_PATH ${SDL2_ROOT_DIR})
endif()

# SYSROOT とかの設定
# これらの設定は project の前に行う必要がある
if (TARGET_OS STREQUAL "linux")
  if (TARGET_ARCH STREQUAL "arm")
    if (TARGET_ARCH_ARM STREQUAL "armv8")
      set(ARCH_NAME aarch64-linux-gnu)
    else()
      set(ARCH_NAME arm-linux-gnueabihf)
    endif()

    set(CMAKE_SYSTEM_NAME Linux)
    if (TARGET_ARCH_ARM STREQUAL "armv8")
      set(CMAKE_SYSTEM_PROCESSOR aarch64)
    else()
      set(CMAKE_SYSTEM_PROCESSOR arm)
    endif()
    set(CMAKE_SYSROOT ${SYSROOT})
    set(CMAKE_C_COMPILER_TARGET ${ARCH_NAME})
    set(CMAKE_CXX_COMPILER_TARGET ${ARCH_NAME})

    set(CMAKE_FIND_ROOT_PATH /root/rootfs)
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
  endif()
endif()


project(momo C CXX)

list(APPEND MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(Boost_USE_STATIC_LIBS ON)
if ("${TARGET_OS}" STREQUAL "windows")
  set(Boost_USE_STATIC_RUNTIME ON)
else()
  set(Boost_USE_STATIC_RUNTIME OFF)
endif()

find_package(WebRTC REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(JSON REQUIRED)
find_package(CLI11 REQUIRED)
if (USE_SDL2)
  find_package(SDL2 REQUIRED)
endif()

add_executable(momo)

string(SUBSTRING "${MOMO_COMMIT}" 0 8 MOMO_COMMIT_SHORT)
string(SUBSTRING "${WEBRTC_COMMIT}" 0 8 WEBRTC_COMMIT_SHORT)
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

target_link_libraries(momo
  PRIVATE
    WebRTC::WebRTC
    Boost::filesystem
    JSON::JSON
    CLI11::CLI11
)

target_compile_definitions(momo
  PRIVATE
    OPENSSL_IS_BORINGSSL
    USE_ROS=$<BOOL:${USE_ROS}>
    USE_NVCODEC_ENCODER=$<BOOL:${USE_NVCODEC_ENCODER}>
    USE_MMAL_ENCODER=$<BOOL:${USE_MMAL_ENCODER}>
    USE_JETSON_ENCODER=$<BOOL:${USE_JETSON_ENCODER}>
    USE_H264=$<BOOL:${USE_H264}>
    USE_SDL2=$<BOOL:${USE_SDL2}>
)

if (USE_SDL2)
  target_sources(momo
    PRIVATE
      src/sdl_renderer/sdl_renderer.cpp
  )
  target_compile_definitions(momo
    PRIVATE
      _THREAD_SAFE
  )

  if (TARGET SDL2::SDL2-static)
    target_link_libraries(momo
      PRIVATE
        SDL2::SDL2-static
        SDL2::SDL2main
    )
  else()
    target_link_libraries(momo PRIVATE SDL2)
    target_include_directories(momo PRIVATE "${SYSROOT}${SDL2_INCLUDE_DIRS}")
  endif()

  if ("${TARGET_OS}" STREQUAL "macos")
    target_link_libraries(momo
      PRIVATE
        iconv
        "-framework Carbon"
        "-framework IOKit"
        "-framework ForceFeedback"
    )
  endif()
endif()

if (USE_ROS)
  target_sources(momo
    PRIVATE
      src/ros/ros_audio_device.cpp
      src/ros/ros_audio_device_module.cpp
      src/ros/ros_video_capture.cpp
  )
  target_compile_definitions(momo PRIVATE HAVE_JPEG=1)
  target_include_directories(momo PRIVATE ${SYSROOT}/opt/ros/${ROS_VERSION}/include)
  target_link_directories(momo PRIVATE ${SYSROOT}/opt/ros/${ROS_VERSION}/lib)
  target_link_libraries(momo
    PRIVATE
      message_filters
      roscpp
      rosconsole
      roscpp_serialization
      rostime
      xmlrpcpp
      cpp_common
      rosconsole_log4cxx
      rosconsole_backend_interface
  )
endif()

if (USE_LIBCXX)
  target_compile_options(momo
    PRIVATE
      "$<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>"
      "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<BOOL:LIBCXX_INCLUDE_DIR>>:-isystem${LIBCXX_INCLUDE_DIR}>"
  )
endif()

if ("${TARGET_OS}" STREQUAL "windows")
  # 文字コードを utf-8 として扱うのと、シンボルテーブル数を増やす
  set_property(TARGET momo APPEND PROPERTY COMPILE_OPTIONS "/utf-8;/bigobj")
  set_target_properties(momo
    PROPERTIES
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

  # NVIDIA Video Codec SDK
  if (USE_NVCODEC_ENCODER)
    target_sources(momo
      PRIVATE
        src/hwenc_nvcodec/nvcodec_h264_encoder.cpp
        NvCodec/NvCodec/NvEncoder/NvEncoder.cpp
        NvCodec/NvCodec/NvEncoder/NvEncoderD3D11.cpp)
    target_include_directories(momo
      PRIVATE
        NvCodec/include
        NvCodec/NvCodec)
    target_link_libraries(momo
      PRIVATE
        DXGI.lib
        D3D11.lib
    )
  endif()

elseif (TARGET_OS STREQUAL "macos")
  target_sources(momo
    PRIVATE
      src/mac_helper/mac_capturer.mm
      src/mac_helper/macos_version.mm
      src/mac_helper/objc_codec_factory_helper.mm
  )

  set_property(TARGET momo APPEND PROPERTY COMPILE_OPTIONS "-fconstant-string-class=NSConstantString")
  set_property(TARGET momo APPEND PROPERTY LINK_OPTIONS "-ObjC")
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

elseif (TARGET_OS STREQUAL "linux")
  find_package(Threads REQUIRED)

  target_sources(momo
    PRIVATE
      src/v4l2_video_capturer/v4l2_video_capturer.cpp
  )
  target_compile_definitions(momo
    PRIVATE
      WEBRTC_POSIX
  )
  if (NOT USE_ROS)
    # USE_ROS=OFF かつ Linux 系の場合はカスタムされた libc++ を使っているためオプション追加
    target_compile_definitions(momo PRIVATE _LIBCPP_ABI_UNSTABLE)
  endif()

  set_target_properties(momo PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(momo
    PRIVATE
      X11
      Xau
      Xdmcp
      xcb
      plds4
      Xext
      expat
      dl
      nss3
      nssutil3
      plc4
      nspr4
      rt
      Threads::Threads
  )

  # NVIDIA Video Codec SDK
  if (USE_NVCODEC_ENCODER)
    target_sources(momo
      PRIVATE
        src/hwenc_nvcodec/nvcodec_h264_encoder.cpp
        src/hwenc_nvcodec/nvcodec_h264_encoder_cuda.cpp
        NvCodec/NvCodec/NvDecoder/NvDecoder.cpp
        NvCodec/NvCodec/NvEncoder/NvEncoder.cpp
        NvCodec/NvCodec/NvEncoder/NvEncoderCuda.cpp)
    target_include_directories(momo
      PRIVATE
        NvCodec/include
        NvCodec/NvCodec
        /usr/local/cuda/include)

    # これらのソースは CUDA としてコンパイルする
    set_source_files_properties(
        src/hwenc_nvcodec/nvcodec_h264_encoder_cuda.cpp
        NvCodec/NvCodec/NvDecoder/NvDecoder.cpp
        NvCodec/NvCodec/NvEncoder/NvEncoderCuda.cpp
      PROPERTIES
        COMPILE_OPTIONS "-xcuda;--cuda-gpu-arch=sm_30"
    )

    target_link_directories(momo PRIVATE /usr/local/cuda/lib64)
    target_link_libraries(momo PRIVATE cuda cudart_static dl rt nvcuvid)
  endif()

  if (TARGET_ARCH STREQUAL "arm")
    if (USE_LIBCXX)
      target_include_directories(momo PRIVATE ${SYSROOT}/usr/include/${ARCH_NAME})
      target_link_directories(momo PRIVATE ${SYSROOT}/usr/lib/${ARCH_NAME})
    endif()

    if (TARGET_ARCH_ARM STREQUAL "armv8")
      # Jetson の設定
      if (USE_JETSON_ENCODER)

        target_sources(momo
          PRIVATE
            src/hwenc_jetson/jetson_h264_encoder.cpp
            src/hwenc_jetson/jetson_video_decoder.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvBuffer.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvElement.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvElementProfiler.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvJpegDecoder.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvJpegEncoder.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvLogging.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvV4l2Element.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvV4l2ElementPlane.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvVideoConverter.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvVideoEncoder.cpp
            ${SYSROOT}/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvVideoDecoder.cpp
        )

        target_include_directories(momo PRIVATE ${SYSROOT}/include/libjpeg-8b)
        target_link_directories(momo PRIVATE ${SYSROOT}/usr/lib/aarch64-linux-gnu/tegra)
        set_property(TARGET momo APPEND PROPERTY LINK_FLAGS "-Wl,-rpath-link=${SYSROOT}/lib/aarch64-linux-gnu")
        set_property(TARGET momo APPEND PROPERTY LINK_FLAGS "-Wl,-rpath-link=${SYSROOT}/usr/lib/aarch64-linux-gnu")
        set_property(TARGET momo APPEND PROPERTY LINK_FLAGS "-Wl,-rpath-link=${SYSROOT}/usr/lib/aarch64-linux-gnu/tegra")
        target_link_libraries(momo
          PRIVATE
            v4l2
            nvbuf_utils
            nvbuf_fdmap
            nvddk_vic
            nvddk_2d_v2
            nvjpeg
            nvrm
            nvrm_graphics
            nvos
        )
      endif(USE_JETSON_ENCODER)
    else()
      # armv6, armv7 用
      if (TARGET_ARCH_ARM STREQUAL "armv6")
        set_property(TARGET momo APPEND PROPERTY COMPILE_OPTIONS "-mfloat-abi=hard;-mcpu=arm1176jzf-s;-mfpu=vfp")
      else()
        set_property(TARGET momo APPEND PROPERTY COMPILE_OPTIONS "-mfloat-abi=hard;-march=armv7-a;-mtune=generic-armv7-a;-mfpu=neon;-mthumb")
      endif()

      if (USE_MMAL_ENCODER)
        target_sources(momo
          PRIVATE
            src/hwenc_mmal/mmal_h264_encoder.cpp
        )

        if (TARGET_OS_LINUX STREQUAL "raspbian-buster")
          set(_VC_PATH ${SYSROOT}/opt/vc)
        else()
          set(_VC_PATH ${SYSROOT}/usr)
        endif()

        target_include_directories(momo PRIVATE ${_VC_PATH}/include)
        set_property(TARGET momo APPEND PROPERTY COMPILE_OPTIONS "-ftree-vectorize;-pipe")
        target_compile_definitions(momo
          PRIVATE
            STANDALONE
            __STDC_CONSTANT_MACROS
            __STDC_LIMIT_MACROS
            TARGET_POSIX
            _LINUX
            PIC
            _REENTRANT
            _LARGEFILE64_SOURCE
            _FILE_OFFSET_BITS=64
            _FORTIFY_SOURCE
            HAVE_LIBOPENMAX=2
            OMX
            OMX_SKIP64BIT
            USE_EXTERNAL_OMX
            HAVE_LIBBCM_HOST
            USE_EXTERNAL_LIBBCM_HOST
            USE_VCHIQ_ARM
        )
        target_link_directories(momo PRIVATE ${_VC_PATH}/lib)
        target_link_libraries(momo
          PRIVATE
            brcmGLESv2
            brcmEGL
            bcm_host
            containers
            vcos
            vcsm
            vchiq_arm
            mmal
            mmal_core
            mmal_components
            mmal_util
            mmal_vc_client
            m
        )

      endif(USE_MMAL_ENCODER)
    endif(TARGET_ARCH_ARM STREQUAL "armv8")
  endif(TARGET_ARCH STREQUAL "arm")
endif()
