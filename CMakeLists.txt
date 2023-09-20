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
set(USE_NVCODEC_ENCODER OFF CACHE BOOL "NVIDIA VIDEO CODEC SDK のハードウェアエンコーダを利用するかどうか")
set(USE_MMAL_ENCODER OFF CACHE BOOL "MMAL ハードウェアエンコーダを利用するかどうか")
set(USE_JETSON_ENCODER OFF CACHE BOOL "Jetson のハードウェアエンコーダを利用するかどうか")
set(USE_MSDK_ENCODER OFF CACHE BOOL "Intel Media SDK のハードウェアエンコーダを利用するかどうか")
set(USE_H264 OFF CACHE BOOL "H264 を利用するかどうか")
set(USE_SDL2 OFF CACHE BOOL "SDL2 による画面出力を利用するかどうか")
set(USE_LINUX_PULSE_AUDIO OFF CACHE BOOL "Linux で ALSA の代わりに PulseAudio を利用するか")
set(USE_SCREEN_CAPTURER OFF CACHE BOOL "スクリーンキャプチャラを利用するかどうか")
set(BOOST_ROOT_DIR "" CACHE PATH "Boost のインストール先ディレクトリ\n空文字だった場合はデフォルト検索パスの Boost を利用する")
set(SDL2_ROOT_DIR "" CACHE PATH "SDL2 のインストール先ディレクトリ\n空文字だった場合はデフォルト検索パスの SDL2 を利用する")
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

if(MOMO_PACKAGE_NAME STREQUAL "windows_x86_64")
  set(_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/windows_x86_64/_install)

  set(TARGET_OS "windows")
  set(TARGET_ARCH "x86_64")
  set(USE_NVCODEC_ENCODER ON)
  set(USE_MSDK_ENCODER ON)
  set(USE_H264 ON)
  set(USE_SDL2 ON)
  set(USE_SCREEN_CAPTURER ON)
  set(BOOST_ROOT_DIR ${_INSTALL_DIR}/boost)
  set(SDL2_ROOT_DIR ${_INSTALL_DIR}/SDL2)
  set(CLI11_ROOT_DIR ${_INSTALL_DIR}/CLI11)
  set(WEBRTC_INCLUDE_DIR ${_INSTALL_DIR}/webrtc/include)
  set(WEBRTC_LIBRARY_DIR ${_INSTALL_DIR}/webrtc/lib)

  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${SDL2_ROOT_DIR}/cmake)
  list(APPEND CMAKE_PREFIX_PATH ${BOOST_ROOT_DIR} ${SDL2_ROOT_DIR})

elseif(MOMO_PACKAGE_NAME STREQUAL "macos_arm64")

  set(_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/macos_arm64/_install)

  set(TARGET_OS "macos")
  set(TARGET_ARCH "arm64")
  set(USE_H264 ON)
  set(USE_SDL2 ON)
  set(USE_SCREEN_CAPTURER ON)
  set(BOOST_ROOT_DIR ${_INSTALL_DIR}/boost)
  set(SDL2_ROOT_DIR ${_INSTALL_DIR}/SDL2)
  set(CLI11_ROOT_DIR ${_INSTALL_DIR}/CLI11)
  set(WEBRTC_INCLUDE_DIR ${_INSTALL_DIR}/webrtc/include)
  set(WEBRTC_LIBRARY_DIR ${_INSTALL_DIR}/webrtc/lib)

elseif(MOMO_PACKAGE_NAME STREQUAL "raspberry-pi-os_armv6")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "raspberry-pi-os")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv6")
  set(USE_MMAL_ENCODER ON)
  set(USE_H264 ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)
  set(CLANG_ROOT /root/llvm/clang)
  set(SYSROOT /root/rootfs)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

  set(Boost_ARCHITECTURE 32)

elseif(MOMO_PACKAGE_NAME STREQUAL "raspberry-pi-os_armv7")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "raspberry-pi-os")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv7")
  set(USE_MMAL_ENCODER ON)
  set(USE_H264 ON)
  set(USE_SDL2 ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(SDL2_ROOT_DIR /root/SDL2)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)
  set(CLANG_ROOT /root/llvm/clang)
  set(SYSROOT /root/rootfs)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

  set(Boost_ARCHITECTURE 32)

elseif(MOMO_PACKAGE_NAME STREQUAL "raspberry-pi-os_armv8")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "raspberry-pi-os")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv8")
  set(USE_V4L2_ENCODER ON)
  set(USE_H264 ON)
  set(USE_SDL2 ON)
  set(USE_LINUX_PULSE_AUDIO ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(SDL2_ROOT_DIR /root/SDL2)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)
  set(CLANG_ROOT /root/llvm/clang)
  set(SYSROOT /root/rootfs)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

elseif(MOMO_PACKAGE_NAME STREQUAL "ubuntu-20.04_x86_64")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-20.04")
  set(TARGET_ARCH "x86_64")
  set(USE_H264 ON)
  set(USE_SDL2 ON)
  set(USE_NVCODEC_ENCODER ON)
  set(USE_MSDK_ENCODER ON)
  set(USE_SCREEN_CAPTURER ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(SDL2_ROOT_DIR /root/SDL2)
  set(LIBVA_ROOT_DIR /root/libva)
  set(MSDK_ROOT_DIR /root/msdk)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)

  # /root/llvm/clang にあるコンパイラは使わず、apt でインストールした clang-10 を利用する
  set(CMAKE_C_COMPILER clang-10)
  set(CMAKE_CXX_COMPILER clang++-10)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

elseif(MOMO_PACKAGE_NAME STREQUAL "ubuntu-20.04_armv8_jetson_xavier")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-20.04")
  set(TARGET_ARCH "arm")
  set(TARGET_ARCH_ARM "armv8")
  set(USE_JETSON_ENCODER ON)
  set(USE_H264 ON)
  set(USE_SDL2 ON)
  set(USE_LINUX_PULSE_AUDIO ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(SDL2_ROOT_DIR /root/SDL2)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)
  set(CLANG_ROOT /root/llvm/clang)
  set(SYSROOT /root/rootfs)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

elseif(MOMO_PACKAGE_NAME STREQUAL "ubuntu-22.04_x86_64")

  set(TARGET_OS "linux")
  set(TARGET_OS_LINUX "ubuntu-22.04")
  set(TARGET_ARCH "x86_64")
  set(USE_H264 ON)
  set(USE_SDL2 ON)
  set(USE_NVCODEC_ENCODER ON)
  set(USE_MSDK_ENCODER ON)
  set(USE_SCREEN_CAPTURER ON)
  set(BOOST_ROOT_DIR /root/boost)
  set(CLI11_ROOT_DIR /root/CLI11)
  set(SDL2_ROOT_DIR /root/SDL2)
  set(LIBVA_ROOT_DIR /root/libva)
  set(MSDK_ROOT_DIR /root/msdk)
  set(WEBRTC_INCLUDE_DIR /root/webrtc/include)
  set(WEBRTC_LIBRARY_DIR /root/webrtc/lib)

  # /root/llvm/clang にあるコンパイラは使わず、apt でインストールした clang-12 を利用する
  set(CMAKE_C_COMPILER clang-12)
  set(CMAKE_CXX_COMPILER clang++-12)

  set(USE_LIBCXX ON)
  set(LIBCXX_INCLUDE_DIR /root/llvm/libcxx/include)

else()
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

    set(CMAKE_FIND_ROOT_PATH ${SYSROOT})
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
  endif()
endif()

if (TARGET_OS STREQUAL "macos")
  set(ARCH_NAME aarch64-apple-darwin)
  set(CMAKE_SYSTEM_PROCESSOR arm64)

  set(CMAKE_C_COMPILER_TARGET ${ARCH_NAME})
  set(CMAKE_CXX_COMPILER_TARGET ${ARCH_NAME})
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
find_package(Boost REQUIRED COMPONENTS filesystem json)
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
    src/ssl_verifier.cpp
    src/ayame/ayame_client.cpp
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
    src/rtc/scalable_track_source.cpp
    src/serial_data_channel/serial_data_channel.cpp
    src/serial_data_channel/serial_data_manager.cpp
    src/sora/sora_server.cpp
    src/sora/sora_session.cpp
    src/sora/sora_client.cpp
    src/websocket.cpp
    src/metrics/metrics_server.cpp
    src/metrics/metrics_session.cpp
)

target_include_directories(momo PRIVATE src)

if ("${TARGET_OS}" STREQUAL "windows")
  set_target_properties(momo PROPERTIES CXX_STANDARD 20 C_STANDARD 99)
else()
  set_target_properties(momo PROPERTIES CXX_STANDARD 17 C_STANDARD 99)
endif()

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
    USE_NVCODEC_ENCODER=$<BOOL:${USE_NVCODEC_ENCODER}>
    USE_MMAL_ENCODER=$<BOOL:${USE_MMAL_ENCODER}>
    USE_V4L2_ENCODER=$<BOOL:${USE_V4L2_ENCODER}>
    USE_JETSON_ENCODER=$<BOOL:${USE_JETSON_ENCODER}>
    USE_MSDK_ENCODER=$<BOOL:${USE_MSDK_ENCODER}>
    USE_H264=$<BOOL:${USE_H264}>
    USE_SDL2=$<BOOL:${USE_SDL2}>
    USE_LINUX_PULSE_AUDIO=$<BOOL:${USE_LINUX_PULSE_AUDIO}>
    USE_SCREEN_CAPTURER=$<BOOL:${USE_SCREEN_CAPTURER}>
    # https://github.com/boostorg/container_hash/issues/22 と同じ問題が clang-15 でも起きるので、これを手動で定義して回避する
    BOOST_NO_CXX98_FUNCTION_BASE
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


  if ("${TARGET_OS}" STREQUAL "windows" OR "${TARGET_OS}" STREQUAL "macos")
    target_link_libraries(momo
      PRIVATE
        SDL2::SDL2-static
        SDL2::SDL2main
    )
  else()
    if (TARGET SDL2::SDL2Config)
      target_link_libraries(momo
        PRIVATE
          SDL2::SDL2Config
      )
    else()
      target_link_directories(momo PRIVATE ${SDL2_LIBDIR})
      target_link_libraries(momo PRIVATE SDL2)
      target_include_directories(momo PRIVATE "${SYSROOT}${SDL2_INCLUDE_DIRS}")
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
      Shcore.lib
      wmcodecdspuuid.lib
      Dwmapi.lib
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
        NvCodec/NvCodec/NvEncoder/NvEncoderD3D11.cpp
        src/hwenc_nvcodec/nvcodec_video_decoder.cpp)
    target_include_directories(momo
      PRIVATE
        NvCodec/include
        NvCodec/NvCodec)
    target_link_libraries(momo
      PRIVATE
        DXGI.lib
        D3D11.lib
    )

    # enable_language(CUDA) は CUDA 用 Visual Studio Integration を利用して CUDA を検出しているが、
    # CUDA ドライバがインストールできない場所では VS Integration をインストールできないため、
    # enable_language(CUDA) は利用できない。
    # なので（deprecated だけど）FindCUDA を利用してコンパイルする。

    set(CUDA_TOOLKIT_ROOT_DIR ${_INSTALL_DIR}/cuda/nvcc)
    find_package(CUDA REQUIRED)

    set_source_files_properties(
        src/cuda/cuda_context_cuda.cpp
        NvCodec/NvCodec/NvDecoder/NvDecoder.cpp
        src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp
      PROPERTIES
        CUDA_SOURCE_PROPERTY_FORMAT OBJ
    )
    cuda_compile(CUDA_FILES
        src/cuda/cuda_context_cuda.cpp
        NvCodec/NvCodec/NvDecoder/NvDecoder.cpp
        src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp
      OPTIONS
        -Xcompiler /utf-8
        -Xcompiler /I${CMAKE_CURRENT_SOURCE_DIR}/NvCodec/include
        -Xcompiler /I${CMAKE_CURRENT_SOURCE_DIR}/NvCodec/NvCodec
        -Xcompiler /I${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_sources(momo PRIVATE ${CUDA_FILES})
    target_include_directories(momo PRIVATE ${CUDA_INCLUDE_DIRS})
    target_link_libraries(momo
      PRIVATE
        ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64/cudart_static.lib 
        #${CUDA_LIBRARIES}
    )
  endif()

  # Intel Media SDK
  if (USE_MSDK_ENCODER)
    target_sources(momo
      PRIVATE
        src/hwenc_msdk/msdk_session.cpp
        src/hwenc_msdk/msdk_video_decoder.cpp
        src/hwenc_msdk/msdk_video_encoder.cpp
    )
    target_include_directories(momo PRIVATE ${_INSTALL_DIR}/msdk/include)
    target_link_libraries(momo
      PRIVATE
        ${_INSTALL_DIR}/msdk/lib/libmfx.lib 
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
      WEBRTC_LINUX
  )
  # Linux 系の場合はカスタムされた libc++ を使っているためオプション追加
  target_compile_definitions(momo PRIVATE _LIBCPP_ABI_NAMESPACE=Cr _LIBCPP_ABI_VERSION=2 _LIBCPP_DISABLE_AVAILABILITY)

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
        src/cuda/cuda_context_cuda.cpp
        src/hwenc_nvcodec/nvcodec_h264_encoder.cpp
        src/hwenc_nvcodec/nvcodec_v4l2_capturer.cpp
        src/hwenc_nvcodec/nvcodec_h264_encoder_cuda.cpp
        src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp
        src/hwenc_nvcodec/nvcodec_video_decoder.cpp
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
        src/cuda/cuda_context_cuda.cpp
        src/hwenc_nvcodec/nvcodec_h264_encoder_cuda.cpp
        src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp
        NvCodec/NvCodec/NvDecoder/NvDecoder.cpp
        NvCodec/NvCodec/NvEncoder/NvEncoderCuda.cpp
      PROPERTIES
        COMPILE_OPTIONS "-xcuda;--cuda-gpu-arch=sm_35;-std=gnu++20"
    )

    target_link_directories(momo PRIVATE /usr/local/cuda/lib64)
    target_link_libraries(momo PRIVATE cudart_static dl rt)
  endif()

  # Intel Media SDK
  if (USE_MSDK_ENCODER)
    find_package(MSDK REQUIRED)
    target_sources(momo
      PRIVATE
        src/hwenc_msdk/msdk_session.cpp
        src/hwenc_msdk/msdk_video_decoder.cpp
        src/hwenc_msdk/msdk_video_encoder.cpp
        src/hwenc_msdk/vaapi_utils_drm.cpp
        src/hwenc_msdk/vaapi_utils.cpp
    )
    target_link_libraries(momo
      PRIVATE
        MSDK::MSDK)
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
            src/hwenc_jetson/jetson_buffer.cpp
            src/hwenc_jetson/jetson_jpeg_decoder.cpp
            src/hwenc_jetson/jetson_jpeg_decoder_pool.cpp
            src/hwenc_jetson/jetson_v4l2_capturer.cpp
            src/hwenc_jetson/jetson_video_encoder.cpp
            src/hwenc_jetson/jetson_video_decoder.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvBufSurface.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvBuffer.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvElement.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvElementProfiler.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvJpegDecoder.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvJpegEncoder.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvLogging.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2Element.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2ElementPlane.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvVideoEncoder.cpp
            ${SYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvVideoDecoder.cpp
        )

        target_include_directories(momo PRIVATE ${SYSROOT}/usr/src/jetson_multimedia_api/include)
        target_include_directories(momo PRIVATE ${SYSROOT}/usr/src/jetson_multimedia_api/include/libjpeg-8b)
        target_link_directories(momo PRIVATE ${SYSROOT}/usr/lib/aarch64-linux-gnu/tegra)
        target_link_options(momo
          PRIVATE
            "-Wl,-rpath-link=${SYSROOT}/lib/aarch64-linux-gnu"
            "-Wl,-rpath-link=${SYSROOT}/usr/lib/aarch64-linux-gnu"
            "-Wl,-rpath-link=${SYSROOT}/usr/lib/aarch64-linux-gnu/tegra"
        )
        target_link_libraries(momo
          PRIVATE
            nvv4l2
            nvv4lconvert
            nvbuf_utils
            #nvbuf_fdmap
            #nvddk_vic
            #nvddk_2d_v2
            nvjpeg
            nvbufsurface
            nvbufsurftransform
            #nvrm
            #nvrm_graphics
            #nvos
        )
      endif(USE_JETSON_ENCODER)

      if (USE_V4L2_ENCODER)
        add_library(camerac SHARED)
        target_include_directories(camerac PRIVATE ${SYSROOT}/usr/include/libcamera)
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
      endif(USE_V4L2_ENCODER)
    else()
      # armv6, armv7 用
      if (TARGET_ARCH_ARM STREQUAL "armv6")
        target_compile_options(momo PRIVATE -mfloat-abi=hard -mcpu=arm1176jzf-s -mfpu=vfp)
      else()
        target_compile_options(momo PRIVATE -mfloat-abi=hard -march=armv7-a -mtune=generic-armv7-a -mfpu=neon -mthumb)
      endif()

      if (USE_MMAL_ENCODER)
        target_sources(momo
          PRIVATE
            src/hwenc_mmal/mmal_h264_decoder.cpp
            src/hwenc_mmal/mmal_buffer.cpp
            src/hwenc_mmal/mmal_h264_encoder.cpp
            src/hwenc_mmal/mmal_v4l2_capturer.cpp
        )

        if (TARGET_OS_LINUX STREQUAL "raspberry-pi-os")
          set(_VC_PATH ${SYSROOT}/opt/vc)
        else()
          set(_VC_PATH ${SYSROOT}/usr)
        endif()

        target_include_directories(momo PRIVATE ${_VC_PATH}/include)
        target_compile_options(momo PRIVATE -ftree-vectorize -pipe)
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
            GLESv2
            EGL
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
