cmake_minimum_required(VERSION 3.27)

# Only interpret if() arguments as variables or keywords when unquoted.
cmake_policy(SET CMP0054 NEW)
# MSVC runtime library flags are selected by an abstraction.
cmake_policy(SET CMP0091 NEW)
# find_package() uses upper-case <PACKAGENAME>_ROOT variables.
cmake_policy(SET CMP0144 NEW)
# The FindCUDA module is removed.
cmake_policy(SET CMP0146 OLD)
# Visual Studio Generators build custom commands in parallel.
cmake_policy(SET CMP0147 OLD)

set(MOMO_TARGET "" CACHE STRING "ビルド設定を自動化するパッケージ名")
set(TARGET_OS "" CACHE STRING "ビルド対象の動作する OS。\n有効な値は windows, macos, linux")
set(TARGET_OS_LINUX "" CACHE STRING "TARGET_OS が linux の場合の詳細な OS の情報。")
set(TARGET_ARCH "" CACHE STRING "ビルド対象の動作する CPU。\n有効な値は x86_64, arm")
set(TARGET_ARCH_ARM "" CACHE STRING "TARGET_ARCH が arm の場合の詳細な CPU の情報。\n有効な値は armv8")
set(USE_NVCODEC_ENCODER OFF CACHE BOOL "NVIDIA VIDEO CODEC SDK のハードウェアエンコーダを利用するかどうか")
set(USE_JETSON_ENCODER OFF CACHE BOOL "Jetson のハードウェアエンコーダを利用するかどうか")
set(USE_VPL_ENCODER OFF CACHE BOOL "Intel VPL のハードウェアエンコーダを利用するかどうか")
set(USE_LINUX_PULSE_AUDIO OFF CACHE BOOL "Linux で ALSA の代わりに PulseAudio を利用するか")
set(USE_SCREEN_CAPTURER OFF CACHE BOOL "スクリーンキャプチャラを利用するかどうか")
set(BOOST_ROOT "" CACHE PATH "Boost のインストール先ディレクトリ\n空文字だった場合はデフォルト検索パスの Boost を利用する")
set(SDL3_ROOT_DIR "" CACHE PATH "SDL3 のインストール先ディレクトリ\n空文字だった場合はデフォルト検索パスの SDL3 を利用する")
set(CLI11_ROOT_DIR "" CACHE PATH "CLI11 のインストール先ディレクトリ")
set(OPENH264_ROOT_DIR "" CACHE PATH "OpenH264 のインストール先ディレクトリ")
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

if(MOMO_TARGET STREQUAL "windows_x86_64")
  set(TARGET_OS "windows")
  set(TARGET_ARCH "x86_64")
elseif(MOMO_TARGET STREQUAL "macos_arm64")
  set(TARGET_OS "macos")
  set(TARGET_ARCH "arm64")
elseif(MOMO_TARGET STREQUAL "raspberry-pi-os_armv8")
  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "raspberry-pi-os")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv8")
elseif(MOMO_TARGET STREQUAL "ubuntu-22.04_x86_64")
  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-22.04")
  set(TARGET_ARCH "x86_64")
elseif(MOMO_TARGET STREQUAL "ubuntu-24.04_x86_64")
  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-24.04")
  set(TARGET_ARCH "x86_64")
elseif(MOMO_TARGET STREQUAL "ubuntu-22.04_armv8_jetson")
  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-22.04")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv8")
endif()

# ubuntu-22.04_armv8_jetson 対応時に追加
# rootfs 上のプログラムが利用されることを防ぐ
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

project(momo C CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_PREFIX_PATH ${SDL3_ROOT_DIR})
if (USE_VPL_ENCODER)
  list(APPEND CMAKE_PREFIX_PATH ${VPL_ROOT_DIR}/lib/cmake/vpl)
endif()

set(Boost_USE_STATIC_LIBS ON)
if (WIN32)
  set(Boost_USE_STATIC_RUNTIME ON)
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

find_package(WebRTC REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem json)
find_package(CLI11 REQUIRED)
find_package(SDL3 REQUIRED CONFIG REQUIRED COMPONENTS SDL3)

set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

add_executable(momo)

string(SUBSTRING "${MOMO_COMMIT}" 0 8 MOMO_COMMIT_SHORT)
string(SUBSTRING "${WEBRTC_COMMIT}" 0 8 WEBRTC_COMMIT_SHORT)
configure_file(src/momo_version.gen.h.template ${CMAKE_CURRENT_BINARY_DIR}/momo_version.gen.h)
target_include_directories(momo PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(momo PRIVATE src/sora-cpp-sdk/include)
target_include_directories(momo PRIVATE ${OPENH264_ROOT_DIR}/include)

target_sources(momo
  PRIVATE
    src/ayame/ayame_client.cpp
    src/main.cpp
    src/metrics/metrics_server.cpp
    src/metrics/metrics_session.cpp
    src/momo_version.cpp
    src/p2p/p2p_server.cpp
    src/p2p/p2p_session.cpp
    src/p2p/p2p_websocket_session.cpp
    src/rtc/aligned_encoder_adapter.cpp
    src/rtc/device_video_capturer.cpp
    src/rtc/momo_video_decoder_factory.cpp
    src/rtc/momo_video_encoder_factory.cpp
    src/rtc/native_buffer.cpp
    src/rtc/peer_connection_observer.cpp
    src/rtc/rtc_connection.cpp
    src/rtc/rtc_manager.cpp
    src/rtc/rtc_ssl_verifier.cpp
    src/serial_data_channel/serial_data_channel.cpp
    src/serial_data_channel/serial_data_manager.cpp
    src/sora-cpp-sdk/src/open_h264_video_encoder.cpp
    src/sora-cpp-sdk/src/scalable_track_source.cpp
    src/sora-cpp-sdk/src/vpl_session_impl.cpp
    src/sora/sora_client.cpp
    src/sora/sora_server.cpp
    src/sora/sora_session.cpp
    src/ssl_verifier.cpp
    src/util.cpp
    src/watchdog.cpp
    src/websocket.cpp
)

target_include_directories(momo PRIVATE src)

set_target_properties(momo PROPERTIES CXX_STANDARD 20 C_STANDARD 99)

target_link_libraries(momo
  PRIVATE
    WebRTC::WebRTC
    Boost::filesystem
    Boost::json
    CLI11::CLI11
)

target_compile_definitions(momo
  PRIVATE
    # CLI11 は C++17 でのビルドにおいて std::filesystem を利用するが、これを clang で解決するには c++fs のリンクが必要になる。
    # しかし、 libwebrtc の libc++ では std::filesystem が提供されていないため下記のオプションを追加して利用しないようにする。
    CLI11_HAS_FILESYSTEM=0
    OPENSSL_IS_BORINGSSL
    $<$<BOOL:${USE_NVCODEC_ENCODER}>:USE_NVCODEC_ENCODER>
    $<$<BOOL:${USE_V4L2_ENCODER}>:USE_V4L2_ENCODER>
    $<$<BOOL:${USE_JETSON_ENCODER}>:USE_JETSON_ENCODER>
    $<$<BOOL:${USE_VPL_ENCODER}>:USE_VPL_ENCODER>
    $<$<BOOL:${USE_LINUX_PULSE_AUDIO}>:USE_LINUX_PULSE_AUDIO>
    $<$<BOOL:${USE_SCREEN_CAPTURER}>:USE_SCREEN_CAPTURER>
    # https://github.com/boostorg/container_hash/issues/22 と同じ問題が clang-15 でも起きるので、これを手動で定義して回避する
    BOOST_NO_CXX98_FUNCTION_BASE
)

# SDL3 のための設定
target_sources(momo
  PRIVATE
    src/sdl_renderer/sdl_renderer.cpp
)
target_compile_definitions(momo
  PRIVATE
    _THREAD_SAFE
)
if ("${TARGET_OS}" STREQUAL "windows" OR "${TARGET_OS}" STREQUAL "macos")
  target_link_libraries(momo
    PRIVATE
      SDL3::SDL3-static
  )
else()
  if (TARGET SDL3::SDL3)
    target_link_libraries(momo
      PRIVATE
        SDL3::SDL3
    )
  else()
    target_link_directories(momo PRIVATE ${SDL3_LIBDIR})
    target_link_libraries(momo PRIVATE SDL3)
    target_include_directories(momo PRIVATE "${SYSROOT}${SDL3_INCLUDE_DIRS}")
  endif()
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

if (USE_LIBCXX)
  target_compile_options(momo
    PRIVATE
      "$<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>"
      "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<BOOL:LIBCXX_INCLUDE_DIR>>:-isystem${LIBCXX_INCLUDE_DIR}>"
  )
endif()

if (USE_SCREEN_CAPTURER)
  target_sources(momo
    PRIVATE
      src/rtc/screen_video_capturer.cpp
  )
  if (TARGET_OS STREQUAL "linux")
    target_link_libraries(momo
      PRIVATE
        # gio-2.0
        # gobject-2.0
        # glib-2.0
        # Xext
        Xdamage
        Xfixes
        Xcomposite
        Xrandr
    )
  endif()
  if (TARGET_OS STREQUAL "macos")
    target_link_libraries(momo
      PRIVATE
        "-framework IOSurface"
    )
  endif()
endif()

if ("${TARGET_OS}" STREQUAL "windows")
  # 文字コードを utf-8 として扱うのと、シンボルテーブル数を増やす
  target_compile_options(momo PRIVATE /utf-8 /bigobj)
  set_target_properties(momo
    PROPERTIES
      # CRTライブラリを静的リンクさせる
      MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
  )

  target_link_libraries(momo
    PRIVATE
      # dbghelp.lib
      # delayimp.lib
      # dnsapi.lib
      # msimg32.lib
      # oleaut32.lib
      # psapi.lib
      # shell32.lib
      # shlwapi.lib
      # usp10.lib
      # version.lib
      # wininet.lib
      winmm.lib
      ws2_32.lib
      # amstrmid.lib
      Strmiids.lib
      # crypt32.lib
      dmoguids.lib
      iphlpapi.lib
      msdmo.lib
      Secur32.lib
      Shcore.lib
      wmcodecdspuuid.lib
      Dwmapi.lib
      D3D11.lib
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
      RTC_ENABLE_H265
  )

  # NVIDIA Video Codec SDK
  if (USE_NVCODEC_ENCODER)
    target_sources(momo
      PRIVATE
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder.cpp
        third_party/NvCodec/NvCodec/NvEncoder/NvEncoder.cpp
        third_party/NvCodec/NvCodec/NvEncoder/NvEncoderD3D11.cpp
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_decoder.cpp)
    target_include_directories(momo
      PRIVATE
        third_party/NvCodec/include
        third_party/NvCodec/NvCodec)
    target_link_libraries(momo
      PRIVATE
        DXGI.lib
        D3D11.lib
    )

    # enable_language(CUDA) は CUDA 用 Visual Studio Integration を利用して CUDA を検出しているが、
    # CUDA ドライバがインストールできない場所では VS Integration をインストールできないため、
    # enable_language(CUDA) は利用できない。
    # なので（deprecated だけど）FindCUDA を利用してコンパイルする。

    find_package(CUDA REQUIRED)

    set_source_files_properties(
        src/sora-cpp-sdk/src/cuda_context_cuda.cpp
        third_party/NvCodec/NvCodec/NvDecoder/NvDecoder.cpp
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp
      PROPERTIES
        CUDA_SOURCE_PROPERTY_FORMAT OBJ
    )
    cuda_compile(CUDA_FILES
        src/sora-cpp-sdk/src/cuda_context_cuda.cpp
        third_party/NvCodec/NvCodec/NvDecoder/NvDecoder.cpp
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp
      OPTIONS
        # VS 2022 の 17.10.x 以上に上げると、ツールセットのバージョンが 14.40 以上になってしまい、以下のエラーが出るため -allow-unsupported-compiler を指定する
        #
        # G:\dev\sora-cpp-sdk\_install\windows_x86_64\release\cuda\include\crt/host_config.h(153): fatal error C1189: #error:  -- unsupported Microsoft Visual Studio version! Only the versions between 20 17 and 2022 (inclusive) are supported! The nvcc flag '-allow-unsupported-compiler' can be used to override this version check; however, using an unsupported host compiler may cause compilation  failure or incorrect run time execution. Use at your own risk. [G:\dev\sora-cpp-sdk\_build\windows_x86_64\release\sora\sora.vcxproj]
        #
        # host_config.h では以下のような記述になっている
        #
        # ```
        # #if _MSC_VER < 1910 || _MSC_VER >= 1940
        #
        # #error -- unsupported Microsoft Visual Studio version! Only the versions between 2017 and 2022 (inclusive) are supported! (...snip...)
        # ```
        #
        # 17.10 は _MSC_VER が 1940 になるため、このエラーが出る。
        #
        # VS のバージョンと _MSC_VER のリストは以下を参照:
        #   https://devblogs.microsoft.com/cppblog/msvc-toolset-minor-version-number-14-40-in-vs-2022-v17-10/
        -allow-unsupported-compiler
        # 更に STL が CUDA 12.4 以上のバージョンを要求するため、STL のバージョンも無視する
        # ref: https://stackoverflow.com/questions/78515942/cuda-compatibility-with-visual-studio-2022-version-17-10
        -Xcompiler /D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH
        -Xcompiler /utf-8
        -Xcompiler /I${CMAKE_CURRENT_SOURCE_DIR}/third_party/NvCodec/include
        -Xcompiler /I${CMAKE_CURRENT_SOURCE_DIR}/third_party/NvCodec/NvCodec
        -Xcompiler /I${CMAKE_CURRENT_SOURCE_DIR}/src
        -Xcompiler /I${CMAKE_CURRENT_SOURCE_DIR}/src/sora-cpp-sdk/include
        -Xcompiler "/MT$<$<CONFIG:Debug>:d>"
        -Xcompiler /D_HAS_ITERATOR_DEBUGGING=0
        -Xcompiler /DUSE_NVCODEC_ENCODER
    )
    target_sources(momo PRIVATE ${CUDA_FILES})
    target_include_directories(momo PRIVATE ${CUDA_INCLUDE_DIRS})
    target_link_libraries(momo
      PRIVATE
        ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64/cudart_static.lib 
        #${CUDA_LIBRARIES}
    )
  endif()

  # Intel VPL
  if (USE_VPL_ENCODER)
    target_sources(momo
      PRIVATE
        src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_decoder.cpp
        src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_encoder.cpp
    )
    target_include_directories(momo PRIVATE ${VPL_ROOT_DIR}/include)
    target_link_libraries(momo
      PRIVATE
        ${VPL_ROOT_DIR}/lib/vpl$<$<CONFIG:Debug>:d>.lib 
    )
  endif()
elseif (TARGET_OS STREQUAL "macos")
  target_sources(momo
    PRIVATE
      src/mac_helper/mac_capturer.mm
      src/mac_helper/macos_version.mm
      src/mac_helper/objc_codec_factory_helper.mm
  )

  target_compile_options(momo PRIVATE -fconstant-string-class=NSConstantString -mmacosx-version-min=11.0)
  target_link_options(momo PRIVATE -ObjC)
  set_target_properties(momo PROPERTIES CXX_VISIBILITY_PRESET hidden)

  target_compile_definitions(momo
    PRIVATE
      WEBRTC_POSIX
      WEBRTC_MAC
      RTC_ENABLE_H265
  )

  target_link_libraries(momo
    PRIVATE
      #"-framework Foundation"
      "-framework AVFoundation"
      #"-framework CoreServices"
      #"-framework CoreFoundation"
      #"-framework AudioUnit"
      "-framework AudioToolbox"
      "-framework CoreAudio"
      "-framework QuartzCore"
      #"-framework CoreGraphics"
      "-framework CoreMedia"
      #"-framework CoreVideo"
      "-framework VideoToolbox"
      "-framework AppKit"
      "-framework Metal"
      "-framework MetalKit"
      "-framework OpenGL"
      "-framework IOSurface"
      "-framework ScreenCaptureKit"
  )

elseif (TARGET_OS STREQUAL "linux")
  find_package(Threads REQUIRED)

  target_sources(momo
    PRIVATE
      src/sora-cpp-sdk/src/v4l2/v4l2_video_capturer.cpp
  )
  target_compile_definitions(momo
    PRIVATE
      WEBRTC_POSIX
      WEBRTC_LINUX
      _LIBCPP_ABI_NAMESPACE=Cr
      _LIBCPP_ABI_VERSION=2
      _LIBCPP_DISABLE_AVAILABILITY
      _LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE
      # https://github.com/boostorg/container_hash/issues/22 と同じ問題が clang-15 でも起きるので、これを手動で定義して回避する
      BOOST_NO_CXX98_FUNCTION_BASE
      RTC_ENABLE_H265
  )

  set_target_properties(momo PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(momo
    PRIVATE
      X11
      # Xau
      # Xdmcp
      Xtst
      # xcb
      # plds4
      Xext
      # expat
      dl
      # nss3
      # nssutil3
      # plc4
      # nspr4
      # rt
      Threads::Threads
  )

  # NVIDIA Video Codec SDK
  if (USE_NVCODEC_ENCODER)
    target_sources(momo
      PRIVATE
        third_party/NvCodec/NvCodec/NvDecoder/NvDecoder.cpp
        third_party/NvCodec/NvCodec/NvEncoder/NvEncoder.cpp
        third_party/NvCodec/NvCodec/NvEncoder/NvEncoderCuda.cpp
        src/sora-cpp-sdk/src/cuda_context_cuda.cpp
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder.cpp
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder_cuda.cpp
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_v4l2_capturer.cpp
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_decoder.cpp)
    target_include_directories(momo
      PRIVATE
        third_party/NvCodec/include
        third_party/NvCodec/NvCodec
        /usr/local/cuda/include)

    # これらのソースは CUDA としてコンパイルする
    set_source_files_properties(
        src/sora-cpp-sdk/src/cuda_context_cuda.cpp
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder_cuda.cpp
        src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp
        third_party/NvCodec/NvCodec/NvDecoder/NvDecoder.cpp
        third_party/NvCodec/NvCodec/NvEncoder/NvEncoderCuda.cpp
      PROPERTIES
        COMPILE_OPTIONS "-xcuda;--cuda-gpu-arch=sm_35;-std=gnu++17"
    )

    target_link_directories(momo PRIVATE /usr/local/cuda/lib64)
    target_link_libraries(momo PRIVATE cudart_static dl rt)
  endif()

  # Intel VPL
  if (USE_VPL_ENCODER)
    find_package(Libdrm REQUIRED)
    find_package(Libva REQUIRED)
    find_package(VPL REQUIRED)
    target_sources(momo
      PRIVATE
        src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_decoder.cpp
        src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_encoder.cpp
    )
    target_link_libraries(momo
      PRIVATE
        VPL::dispatcher
        Libdrm::drm
        Libva::va
        Libva::va_drm
    )
  endif()

  if (TARGET_ARCH STREQUAL "arm")
    target_include_directories(momo PRIVATE ${CMAKE_SYSROOT}/usr/include/${ARCH_NAME})
    target_link_directories(momo PRIVATE ${CMAKE_SYSROOT}/usr/lib/${ARCH_NAME})

    if (TARGET_ARCH_ARM STREQUAL "armv8")
      # Jetson の設定
      if (USE_JETSON_ENCODER)

        target_sources(momo
          PRIVATE
            src/sora-cpp-sdk/src/hwenc_jetson/jetson_buffer.cpp
            src/sora-cpp-sdk/src/hwenc_jetson/jetson_jpeg_decoder.cpp
            src/sora-cpp-sdk/src/hwenc_jetson/jetson_jpeg_decoder_pool.cpp
            src/sora-cpp-sdk/src/hwenc_jetson/jetson_v4l2_capturer.cpp
            src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp
            src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_decoder.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvBufSurface.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvBuffer.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvElement.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvElementProfiler.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvJpegDecoder.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvJpegEncoder.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvLogging.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2Element.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2ElementPlane.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvVideoEncoder.cpp
            ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvVideoDecoder.cpp
        )

        target_include_directories(momo PRIVATE ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/include)
        target_include_directories(momo PRIVATE ${CMAKE_SYSROOT}/usr/src/jetson_multimedia_api/include/libjpeg-8b)
        target_link_directories(momo PRIVATE ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/nvidia)
        target_link_options(momo
          PRIVATE
            "-Wl,-rpath-link=${CMAKE_SYSROOT}/lib/aarch64-linux-gnu"
            "-Wl,-rpath-link=${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu"
            "-Wl,-rpath-link=${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/nvidia"
        )
        target_link_libraries(momo
          PRIVATE
            nvv4l2
            nvv4lconvert
            #nvbuf_utils
            nvbuf_fdmap
            #nvddk_vic
            #nvddk_2d_v2
            nvjpeg
            nvbufsurface
            nvbufsurftransform
            #nvrm
            #nvrm_graphics
            #nvos
        )
      endif()

      if (USE_V4L2_ENCODER)
        add_library(camerac SHARED)
        target_include_directories(camerac PRIVATE ${CMAKE_SYSROOT}/usr/include/libcamera)
        set_target_properties(camerac PROPERTIES CXX_STANDARD 17 C_STANDARD 99)
        target_sources(camerac
          PRIVATE
            src/libcamerac/libcamerac.cpp
        )
        target_link_libraries(camerac
          PRIVATE
            camera
            camera-base
        )

        target_sources(momo
          PRIVATE
            src/hwenc_v4l2/libcamera_capturer.cpp
            src/hwenc_v4l2/v4l2_buffers.cpp
            src/hwenc_v4l2/v4l2_capturer.cpp
            src/hwenc_v4l2/v4l2_converter.cpp
            src/hwenc_v4l2/v4l2_h264_decoder.cpp
            src/hwenc_v4l2/v4l2_h264_encoder.cpp
            src/hwenc_v4l2/v4l2_native_buffer.cpp
            src/hwenc_v4l2/v4l2_runner.cpp
        )
        target_link_libraries(momo PRIVATE camerac)
        set_target_properties(momo PROPERTIES BUILD_RPATH_USE_ORIGIN ON)
      endif()
    endif()
  endif()
endif()
