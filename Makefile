include VERSION

# momo バイナリを生成するための Makefile

# 外から指定可能な変数の一覧。
#
# 以下の一覧の値は PACKAGE_NAME を指定していれば自動的に設定されますが、
# それぞれを外から指定することも可能です。
#
# TARGET_OS: ビルド対象の動作する OS
#   有効な値は linux, macos
#
# TARGET_OS_LINUX: TARGET_OS が linux の場合の詳細な OS の情報
#   有効な値は raspbian-buster, ubuntu-16.04, ubuntu-18.04
#
# TARGET_ARCH: ビルド対象の動作する CPU
#   有効な値は x86_64, arm
#
# TARGET_ARCH_ARM: TARGET_ARCH が arm の場合の詳細な CPU の情報
#   有効な値は armv6, armv7, armv8
#
# USE_ROS: ROS を使っているかどうか
#   有効な値は 0, 1
#
# USE_MMAL_ENCODER: MMAL ハードウェアエンコーダを利用するかどうか
#   有効な値は 0, 1
#
# USE_JETSON_ENCODER: Jetson のハードウェアエンコーダを利用するかどうか
#   有効な値は 0, 1
#
# USE_H264: H264 を利用するかどうか
#   有効な値は 0, 1
#
# USE_SDL2: SDL2 による画面出力を利用するかどうか
#   有効な値は 0, 1
#
# SDL2_ROOT: SDL2 のインストール先ディレクトリ
#
# BOOST_ROOT: Boost のインストール先ディレクトリ
#
# JSON_ROOT: nlohmann/json のインストール先ディレクトリ
#
# CLI11_ROOT: CLI11 のインストール先ディレクトリ
#
# WEBRTC_INCLUDE_DIR: WebRTC のインクルードディレクトリ
#
# WEBRTC_LIBRARY_DIR: WebRTC のライブラリディレクトリ
#
# WEBRTC_VERSION_FILE: WebRTC のバージョンファイル
#
# CLANG_ROOT: ビルドに使う clang コンパイラのインストール先ディレクトリ
#
# USE_LIBCXX: libstdc++ の代わりに libc++ を使うかどうか
#   有効な値は 0, 1
#
# LIBCXX_INCLUDE_DIR: libc++ を使う場合の libc++ のインクルードディレクトリ
#
# SYSROOT: TARGET_ARCH が arm の場合のクロスビルド用の RootFS ディレクトリ
#
# BUILD_MODE: ビルドモード。明示的に指定しない限り build になる。
#   有効な値は build, package
#
# BUILD_ROOT: ビルド用ディレクトリ。デフォルトでは ../momo-build になる。
#
# MOMO_VERSION: バージョン情報。設定しなければ internal-build になる
#
# MOMO_COMMIT_SHORT: Momo のコミットハッシュ。設定しなければ unknown になる
#
# MOMO_CFLAGS: C コンパイラに追加で渡すフラグ。最適化フラグやデバッグフラグを入れることを想定している。
#
# MOMO_LDFLAGS: C リンカーに追加で渡すフラグ。サニタイズフラグや追加のリンクオブジェクトを入れることを想定している。

ifeq ($(PACKAGE_NAME),raspbian-buster_armv6)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= raspbian-buster
  TARGET_ARCH ?= arm
  TARGET_ARCH_ARM ?= armv6
  USE_ROS ?= 0
  USE_MMAL_ENCODER ?= 1
  USE_JETSON_ENCODER ?= 0
  USE_H264 ?= 1
  USE_SDL2 ?= 0
  BOOST_ROOT ?= /root/boost
  JSON_ROOT ?= /root/json
  CLI11_ROOT ?= /root/CLI11
  WEBRTC_INCLUDE_DIR ?= /root/webrtc/include
  WEBRTC_LIBRARY_DIR ?= /root/webrtc/lib
  WEBRTC_VERSION_FILE ?= /root/webrtc/VERSIONS
  CLANG_ROOT ?= /root/llvm/clang
  USE_LIBCXX ?= 1
  LIBCXX_INCLUDE_DIR ?= /root/llvm/libcxx/include
  SYSROOT ?= /root/rootfs
else ifeq ($(PACKAGE_NAME),raspbian-buster_armv7)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= raspbian-buster
  TARGET_ARCH ?= arm
  TARGET_ARCH_ARM ?= armv7
  USE_ROS ?= 0
  USE_MMAL_ENCODER ?= 1
  USE_JETSON_ENCODER ?= 0
  USE_H264 ?= 1
  USE_SDL2 ?= 1
  BOOST_ROOT ?= /root/boost
  JSON_ROOT ?= /root/json
  CLI11_ROOT ?= /root/CLI11
  WEBRTC_INCLUDE_DIR ?= /root/webrtc/include
  WEBRTC_LIBRARY_DIR ?= /root/webrtc/lib
  WEBRTC_VERSION_FILE ?= /root/webrtc/VERSIONS
  CLANG_ROOT ?= /root/llvm/clang
  USE_LIBCXX ?= 1
  LIBCXX_INCLUDE_DIR ?= /root/llvm/libcxx/include
  SYSROOT ?= /root/rootfs
else ifeq ($(PACKAGE_NAME),ubuntu-16.04_armv7_ros)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= ubuntu-16.04
  TARGET_ARCH ?= arm
  TARGET_ARCH_ARM ?= armv7
  USE_ROS ?= 1
  USE_MMAL_ENCODER ?= 1
  USE_JETSON_ENCODER ?= 0
  USE_H264 ?= 1
  USE_SDL2 ?= 0
  BOOST_ROOT ?= /root/boost
  JSON_ROOT ?= /root/json
  CLI11_ROOT ?= /root/CLI11
  WEBRTC_INCLUDE_DIR ?= /root/webrtc/include
  WEBRTC_LIBRARY_DIR ?= /root/webrtc/lib
  WEBRTC_VERSION_FILE ?= /root/webrtc/VERSIONS
  CLANG_ROOT ?= /root/llvm/clang
  USE_LIBCXX ?= 0
  SYSROOT ?= /root/rootfs
else ifeq ($(PACKAGE_NAME),ubuntu-18.04_armv8)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= ubuntu-18.04
  TARGET_ARCH ?= arm
  TARGET_ARCH_ARM ?= armv8
  USE_ROS ?= 0
  USE_MMAL_ENCODER ?= 0
  USE_JETSON_ENCODER ?= 0
  USE_H264 ?= 0
  USE_SDL2 ?= 0
  BOOST_ROOT ?= /root/boost
  JSON_ROOT ?= /root/json
  CLI11_ROOT ?= /root/CLI11
  WEBRTC_INCLUDE_DIR ?= /root/webrtc/include
  WEBRTC_LIBRARY_DIR ?= /root/webrtc/lib
  WEBRTC_VERSION_FILE ?= /root/webrtc/VERSIONS
  CLANG_ROOT ?= /root/llvm/clang
  USE_LIBCXX ?= 1
  LIBCXX_INCLUDE_DIR ?= /root/llvm/libcxx/include
  SYSROOT ?= /root/rootfs
else ifeq ($(PACKAGE_NAME),ubuntu-18.04_armv8_jetson_nano)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= ubuntu-18.04
  TARGET_ARCH ?= arm
  TARGET_ARCH_ARM ?= armv8
  USE_ROS ?= 0
  USE_MMAL_ENCODER ?= 0
  USE_JETSON_ENCODER ?= 1
  USE_H264 ?= 1
  USE_SDL2 ?= 1
  SDL2_ROOT ?= /root/sdl2
  BOOST_ROOT ?= /root/boost
  JSON_ROOT ?= /root/json
  CLI11_ROOT ?= /root/CLI11
  WEBRTC_INCLUDE_DIR ?= /root/webrtc/include
  WEBRTC_LIBRARY_DIR ?= /root/webrtc/lib
  WEBRTC_VERSION_FILE ?= /root/webrtc/VERSIONS
  CLANG_ROOT ?= /root/llvm/clang
  USE_LIBCXX ?= 1
  LIBCXX_INCLUDE_DIR ?= /root/llvm/libcxx/include
  SYSROOT ?= /root/rootfs
else ifeq ($(PACKAGE_NAME),ubuntu-16.04_x86_64_ros)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= ubuntu-16.04
  TARGET_ARCH ?= x86_64
  USE_ROS ?= 1
  USE_MMAL_ENCODER ?= 0
  USE_JETSON_ENCODER ?= 0
  USE_H264 ?= 0
  USE_SDL2 ?= 0
  BOOST_ROOT ?= /root/boost
  JSON_ROOT ?= /root/json
  CLI11_ROOT ?= /root/CLI11
  WEBRTC_INCLUDE_DIR ?= /root/webrtc/include
  WEBRTC_LIBRARY_DIR ?= /root/webrtc/lib
  WEBRTC_VERSION_FILE ?= /root/webrtc/VERSIONS
  CLANG_ROOT ?= /root/llvm/clang
  USE_LIBCXX ?= 0
else ifeq ($(PACKAGE_NAME),ubuntu-18.04_x86_64)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= ubuntu-18.04
  TARGET_ARCH ?= x86_64
  USE_ROS ?= 0
  USE_MMAL_ENCODER ?= 0
  USE_JETSON_ENCODER ?= 0
  USE_H264 ?= 0
  USE_SDL2 ?= 1
  BOOST_ROOT ?= /root/boost
  JSON_ROOT ?= /root/json
  CLI11_ROOT ?= /root/CLI11
  WEBRTC_INCLUDE_DIR ?= /root/webrtc/include
  WEBRTC_LIBRARY_DIR ?= /root/webrtc/lib
  WEBRTC_VERSION_FILE ?= /root/webrtc/VERSIONS
  CLANG_ROOT ?= /root/llvm/clang
  USE_LIBCXX ?= 1
  LIBCXX_INCLUDE_DIR ?= /root/llvm/libcxx/include
else ifeq ($(PACKAGE_NAME),macos)
  TARGET_OS ?= macos
  TARGET_ARCH ?= x86_64
  USE_ROS ?= 0
  USE_MMAL_ENCODER ?= 0
  USE_JETSON_ENCODER ?= 0
  USE_H264 ?= 1
  USE_SDL2 ?= 1
  SDL2_ROOT ?= $(CURDIR)/build/macos/sdl2
  BOOST_ROOT ?= $(CURDIR)/build/macos/boost
  JSON_ROOT ?= $(CURDIR)/build/macos/json
  CLI11_ROOT ?= $(CURDIR)/build/macos/CLI11
  WEBRTC_INCLUDE_DIR ?= $(CURDIR)/build/macos/webrtc/include
  WEBRTC_LIBRARY_DIR ?= $(CURDIR)/build/macos/webrtc/lib
  WEBRTC_VERSION_FILE ?= $(CURDIR)/build/macos/webrtc/VERSIONS
  CLANG_ROOT ?= $(CURDIR)/build/macos/llvm/clang
  USE_LIBCXX ?= 1
  LIBCXX_INCLUDE_DIR ?= $(CURDIR)/build/macos/llvm/libcxx/include
else
  # 各変数がちゃんと設定されているか確認する

  ifndef TARGET_OS
    $(info - TARGET_OS が指定されていません)
    HAS_ERROR = 1
  endif

  ifeq ($(TARGET_OS),linux)
    ifndef TARGET_OS_LINUX
      $(info - TARGET_OS_LINUX が指定されていません)
      HAS_ERROR = 1
    endif
  endif

  ifndef TARGET_ARCH
    $(info - TARGET_ARCH が指定されていません)
    HAS_ERROR = 1
  endif

  ifeq ($(TARGET_ARCH),arm)
    ifndef TARGET_ARCH_ARM
      $(info - TARGET_ARCH_ARM が指定されていません)
      HAS_ERROR = 1
    endif
  endif

  ifndef USE_ROS
    $(info - USE_ROS が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef USE_MMAL_ENCODER
    $(info - USE_MMAL_ENCODER が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef USE_JETSON_ENCODER
    $(info - USE_JETSON_ENCODER が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef USE_H264
    $(info - USE_H264 が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef USE_SDL2
    $(info - USE_SDL2 が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef BOOST_ROOT
    $(info - BOOST_ROOT が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef WEBRTC_INCLUDE_DIR
    $(info - WEBRTC_INCLUDE_DIR が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef WEBRTC_LIBRARY_DIR
    $(info - WEBRTC_LIBRARY_DIR が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef CLANG_ROOT
    $(info - CLANG_ROOT が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef USE_LIBCXX
    $(info - USE_LIBCXX が指定されていません)
    HAS_ERROR = 1
  endif

  ifeq ($(USE_LIBCXX),1)
    ifndef LIBCXX_INCLUDE_DIR
      $(info - LIBCXX_INCLUDE_DIR が指定されていません)
      HAS_ERROR = 1
    endif
  endif

  ifeq ($(TARGET_ARCH),arm)
    ifndef SYSROOT
      $(info - SYSROOT が指定されていません)
      HAS_ERROR = 1
    endif
  endif

  ifeq ($(HAS_ERROR),1)
    $(info )
    $(info 各変数の設定か、PACKAGE_NAME の設定をして下さい。)
  endif
endif
BUILD_MODE ?= build

ifdef PACKAGE_NAME
  # この BUILD_ROOT のデフォルト値に他の場所でも依存してるので、
  # 書き換える時は気をつけること
  BUILD_ROOT ?= _build/$(PACKAGE_NAME)
else
  BUILD_ROOT ?= _build/local
endif

CFLAGS += -Wno-implicit-int-float-conversion -Wno-macro-redefined -fno-lto -std=c++14 -pthread -DWEBRTC_POSIX -DOPENSSL_IS_BORINGSSL -Isrc/
CFLAGS += -I$(WEBRTC_INCLUDE_DIR) -I$(WEBRTC_INCLUDE_DIR)/third_party/libyuv/include -I$(WEBRTC_INCLUDE_DIR)/third_party/abseil-cpp
LDFLAGS += -L$(WEBRTC_LIBRARY_DIR) -lpthread
ifdef MOMO_VERSION
  CFLAGS += -DMOMO_VERSION='"$(MOMO_VERSION)"'
endif
ifdef MOMO_COMMIT_SHORT
  CFLAGS += -DMOMO_COMMIT_SHORT='"$(MOMO_COMMIT_SHORT)"'
endif

# WebRTC のバージョンファイルがある場合、必要な情報を CFLAGS に追加していく
ifdef WEBRTC_VERSION_FILE
  WEBRTC_READABLE_VERSION = $(shell . $(WEBRTC_VERSION_FILE) && echo $$WEBRTC_READABLE_VERSION)
  WEBRTC_COMMIT_SHORT = $(shell . $(WEBRTC_VERSION_FILE) && echo $$WEBRTC_COMMIT | cut -b 1-8)
  WEBRTC_BUILD_VERSION = $(shell . $(WEBRTC_VERSION_FILE) && echo $$WEBRTC_BUILD_VERSION)
endif
ifdef WEBRTC_READABLE_VERSION
  CFLAGS += -DWEBRTC_READABLE_VERSION='"$(WEBRTC_READABLE_VERSION)"'
endif
ifdef WEBRTC_COMMIT_SHORT
  CFLAGS += -DWEBRTC_COMMIT_SHORT='"$(WEBRTC_COMMIT_SHORT)"'
endif
ifdef WEBRTC_BUILD_VERSION
  CFLAGS += -DWEBRTC_BUILD_VERSION='"$(WEBRTC_BUILD_VERSION)"'
endif

ifeq ($(USE_LIBCXX),1)
  LDFLAGS += -lwebrtc
else
  # libc++ を使わないバージョンの libwebrtc.a を利用する
  LDFLAGS += -lwebrtc_nolibcxx
endif

ifeq ($(USE_ROS),1)
  ifeq ($(TARGET_OS_LINUX),ubuntu-16.04)
    ROS_VERSION = kinetic
  else ifeq ($(TARGET_OS_LINUX),ubuntu-18.04)
    ROS_VERSION = melodic
  else
    $(error ROS_VERSION が決定できません)
  endif
endif

CFLAGS += -DUSE_H264=$(USE_H264)

ifeq ($(TARGET_OS),linux)
  CFLAGS += -fpic
  LDFLAGS += -lX11 -lXau -lXdmcp -lxcb -lplds4 -lXext -lexpat -ldl -lnss3 -lnssutil3 -lplc4 -lnspr4 -lrt

  CC = $(CLANG_ROOT)/bin/clang
  CXX = $(CLANG_ROOT)/bin/clang++
  AR = $(CLANG_ROOT)/bin/llvm-ar

  SOURCES += $(shell find src/v4l2_video_capturer -maxdepth 1 -name '*.cpp')

  ifeq ($(TARGET_ARCH),arm)
    ifeq ($(TARGET_ARCH_ARM),armv8)
      ARCH_NAME = aarch64-linux-gnu
    else
      ARCH_NAME = arm-linux-gnueabihf
    endif

    CC += --sysroot=$(SYSROOT) --target=$(ARCH_NAME)
    CXX += --sysroot=$(SYSROOT) --target=$(ARCH_NAME)
    ifeq ($(USE_ROS),1)
      # USE_ROS=1 の場合は libstdc++ を使う（追加オプション無し）
    else
      # USE_ROS=0 の場合は libc++ を使う
      CFLAGS += -I$(SYSROOT)/usr/include/$(ARCH_NAME)
      LDFLAGS += -L$(SYSROOT)/usr/lib/$(ARCH_NAME)
    endif

    ifeq ($(TARGET_ARCH_ARM),armv8)
      # Jetson の設定
      ifeq ($(USE_JETSON_ENCODER),1)
        CFLAGS += \
          -DUSE_JETSON_ENCODER=1 \
          -I$(SYSROOT)/include/libjpeg-8b
        LDFLAGS += \
          -lv4l2 \
          -L$(SYSROOT)/usr/lib/aarch64-linux-gnu/tegra \
          -Wl,-rpath-link=$(SYSROOT)/lib/aarch64-linux-gnu \
          -Wl,-rpath-link=$(SYSROOT)/usr/lib/aarch64-linux-gnu \
          -Wl,-rpath-link=$(SYSROOT)/usr/lib/aarch64-linux-gnu/tegra \
          -lnvbuf_utils \
          -lnvbuf_fdmap \
          -lnvddk_vic \
          -lnvddk_2d_v2 \
          -lnvjpeg \
          -lnvrm \
          -lnvrm_graphics \
          -lnvos
        SOURCES += $(shell find src/hwenc_jetson -maxdepth 1 -name '*.cpp')
        JETSON_ADDITIONAL_SOURCES += \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvBuffer.cpp \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvElement.cpp \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvElementProfiler.cpp \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvJpegDecoder.cpp \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvJpegEncoder.cpp \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvLogging.cpp \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvV4l2Element.cpp \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvV4l2ElementPlane.cpp \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvVideoConverter.cpp \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvVideoEncoder.cpp \
          $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/samples/common/classes/NvVideoDecoder.cpp
      endif
    else
      # armv6, armv7 用

      CFLAGS += -mfloat-abi=hard
      ifeq ($(TARGET_ARCH_ARM),armv6)
        CFLAGS += -mcpu=arm1176jzf-s -mfpu=vfp
      else
        CFLAGS += -march=armv7-a -mtune=generic-armv7-a -mfpu=neon -mthumb
      endif

      # ilclient の設定
      ifeq ($(USE_MMAL_ENCODER),1)
        ifeq ($(TARGET_OS_LINUX),raspbian-buster)
          VC_PATH = $(SYSROOT)/opt/vc
        else
          VC_PATH = $(SYSROOT)/usr
        endif

        MMAL_CFLAGS = \
          -DSTANDALONE \
          -D__STDC_CONSTANT_MACROS \
          -D__STDC_LIMIT_MACROS \
          -DTARGET_POSIX \
          -D_LINUX \
          -fPIC \
          -DPIC \
          -D_REENTRANT \
          -D_LARGEFILE64_SOURCE \
          -D_FILE_OFFSET_BITS=64 \
          -U_FORTIFY_SOURCE \
          -g \
          -DHAVE_LIBOPENMAX=2 \
          -DOMX \
          -DOMX_SKIP64BIT \
          -ftree-vectorize \
          -pipe \
          -DUSE_EXTERNAL_OMX \
          -DHAVE_LIBBCM_HOST \
          -DUSE_EXTERNAL_LIBBCM_HOST \
          -DUSE_VCHIQ_ARM
        CFLAGS += \
          -DUSE_MMAL_ENCODER=1 \
          $(MMAL_CFLAGS) \
          -I$(VC_PATH)/include/
        LDFLAGS += \
          -L$(VC_PATH)/lib/ \
          -lbrcmGLESv2 \
          -lbrcmEGL \
          -lbcm_host \
          -lcontainers \
          -lvcos \
          -lvcsm \
          -lvchiq_arm \
          -lmmal \
          -lmmal_core \
          -lmmal_components \
          -lmmal_util \
          -lmmal_vc_client \
          -lm
        SOURCES += $(shell find src/hwenc_mmal -maxdepth 1 -name '*.cpp')
      endif
    endif
  endif
endif

ifeq ($(TARGET_OS),macos)
  CC = $(CLANG_ROOT)/bin/clang
  CXX = $(CLANG_ROOT)/bin/clang++
  # brew でインストールした ar コマンドを使うとエラーになるので、明示的にフルパスを指定する
  AR = /usr/bin/ar

  SDK_PATH = $(shell xcrun --sdk macosx --show-sdk-path)
  CC += --sysroot=$(SDK_PATH)
  CXX += --sysroot=$(SDK_PATH)

  CFLAGS += -DWEBRTC_POSIX -DWEBRTC_MAC
  CFLAGS += -fconstant-string-class=NSConstantString -I$(WEBRTC_INCLUDE_DIR)/sdk/objc -I$(WEBRTC_INCLUDE_DIR)/sdk/objc/base
  CFLAGS += -fvisibility=hidden
  LDFLAGS += \
    -ObjC \
    -F$(SDK_PATH)/System/Library/Frameworks \
    -ldl \
    -framework Foundation \
    -framework AVFoundation \
    -framework CoreServices \
    -framework CoreFoundation \
    -framework AudioUnit \
    -framework AudioToolbox \
    -framework CoreAudio \
    -framework CoreGraphics \
    -framework CoreMedia \
    -framework CoreVideo \
    -framework VideoToolbox \
    -framework AppKit \
    -framework Metal \
    -framework MetalKit \
    -framework OpenGL
  ifeq ($(USE_SDL2),1)
    LDFLAGS += \
      -liconv \
      -framework Carbon \
      -framework IOKit \
      -framework ForceFeedback
  endif
  SOURCES += $(shell find src -name '*.mm')
else
  ifeq ($(USE_ROS),0)
    # USE_ROS=0 かつ mac 以外の場合はカスタムされた libc++ を使っているためオプション追加 
    CFLAGS += -D_LIBCPP_ABI_UNSTABLE
  endif
endif

ifeq ($(USE_LIBCXX),1)
  # libc++ を使う
  CFLAGS += -nostdinc++ -isystem$(LIBCXX_INCLUDE_DIR)
endif

ifeq ($(USE_SDL2),1)
  ifdef SDL2_ROOT
    CFLAGS += -I$(SDL2_ROOT)/include
    LDFLAGS += -L$(SDL2_ROOT)/lib
  endif
  CFLAGS += -D_THREAD_SAFE
  LDFLAGS += -lSDL2
  SOURCES += $(shell find src/sdl_renderer -name '*.cpp')
endif
CFLAGS += -DUSE_SDL2=$(USE_SDL2)

SOURCES += $(shell find src -maxdepth 1 -name '*.cpp')
SOURCES += $(shell find src/p2p -name '*.cpp')
SOURCES += $(shell find src/rtc -name '*.cpp')
SOURCES += $(shell find src/ayame -name '*.cpp')
SOURCES += $(shell find src/sora -name '*.cpp')
SOURCES += $(shell find src/ws -name '*.cpp')
SOURCES += $(shell find src/serial_data_channel -maxdepth 1 -name '*.cpp')

ifeq ($(USE_ROS),1)
  CFLAGS += -DHAVE_JPEG=1 -DUSE_ROS=1 -I$(SYSROOT)/opt/ros/$(ROS_VERSION)/include
  LDFLAGS += \
    -lpthread \
    -L$(SYSROOT)/opt/ros/$(ROS_VERSION)/lib \
    -lmessage_filters \
    -lroscpp \
    -lrosconsole \
    -lroscpp_serialization \
    -lrostime \
    -lxmlrpcpp \
    -lcpp_common \
    -lrosconsole_log4cxx \
    -lrosconsole_backend_interface
  SOURCES += $(shell find src/ros -name '*.cpp')
endif

OBJECTS = $(addprefix $(BUILD_ROOT)/,$(patsubst %.mm,%.o,$(patsubst %.cpp,%.o,$(SOURCES))))

# Boost
CFLAGS += -I$(BOOST_ROOT)/include
LDFLAGS += -L$(BOOST_ROOT)/lib -lboost_filesystem
# Boost.Beast で BoringSSL を使うので、そのあたりも追加する
CFLAGS += -I$(WEBRTC_INCLUDE_DIR)/third_party/boringssl/src/include -DOPENSSL_IS_BORINGSSL

# JSON
CFLAGS += -I$(JSON_ROOT)/include

# CLI11
CFLAGS += -I$(CLI11_ROOT)/include

# パッケージ用のフラグ
ifeq ($(BUILD_MODE),package)
  LDFLAGS += -s
endif

# ヘッダの依存関係をうまくやる
DEPS=$(addprefix $(BUILD_ROOT)/,$(patsubst %.mm,%.d,$(patsubst %.cpp,%.d,$(SOURCES))))
CFLAGS += -MMD -MP
-include $(DEPS)

# ユーザ指定のフラグを追加
CFLAGS += $(MOMO_CFLAGS)

# ユーザ指定のフラグを追加
LDFLAGS += $(MOMO_LDFLAGS)

.PHONY: help
help:
	@echo ""
	@echo "momo バイナリを生成するための Makefile"
	@echo ""
	@echo "使い方:"
	@echo "  make PACKAGE_NAME=<パッケージ名> momo"
	@echo ""
	@echo "パッケージ名の一覧については cd build && make help を参照して下さい。"
	@echo ""
	@echo "PACKAGE_NAME の代わりにいくつかの変数を定義することでカスタマイズもできます。"
	@echo "詳細については Makefile のコメントを参照して下さい。"
	@echo ""

$(BUILD_ROOT):
	# ビルド用ディレクトリを作っておく
	mkdir -p $(BUILD_ROOT)

# Jetson Nano を利用する場合の追加ソースのコンパイル
ifeq ($(USE_JETSON_ENCODER),1)

$(BUILD_ROOT)/tegra_multimedia_api/%.o: $(SYSROOT)/usr/src/nvidia/tegra_multimedia_api/%.cpp | $(BUILD_ROOT)
	@mkdir -p `dirname $@`
	$(CXX) $(CFLAGS) $(INCLUDES) -c $< -o $@

# OBJECTS にも追加する
OBJECTS += $(addprefix $(BUILD_ROOT)/,$(subst $(SYSROOT)/usr/src/nvidia/,,$(patsubst %.cpp,%.o,$(JETSON_ADDITIONAL_SOURCES))))

endif

# src 以下のソースのビルドルール
$(BUILD_ROOT)/src/%.o: src/%.cpp | $(BUILD_ROOT)
	@mkdir -p `dirname $@`
	$(CXX) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_ROOT)/src/%.o: src/%.mm | $(BUILD_ROOT)
	@mkdir -p `dirname $@`
	$(CXX) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_ROOT)/momo: $(OBJECTS) | $(BUILD_ROOT)
	$(CXX) -o $(BUILD_ROOT)/momo $(OBJECTS) $(LDFLAGS)

.PHONY: momo
momo:
	# momo 本体のビルド
	$(MAKE) $(BUILD_ROOT)/momo

	# ビルド後に ./momo test で動作確認できるようにしたいので、生成されたバイナリをルートディレクトリにコピーする
	cp $(BUILD_ROOT)/momo momo

.PHONY: clean
clean:
	rm -f $(shell find $(BUILD_ROOT) -type f -name '*.o')
	rm -f $(shell find $(BUILD_ROOT) -type f -name '*.a')
	rm -f $(shell find $(BUILD_ROOT) -type f -name '*.d')
	rm -f momo
