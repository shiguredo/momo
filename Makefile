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
#   有効な値は raspbian-stretch, ubuntu-16.04, ubuntu-18.04
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
# USE_IL_ENCODER: ハードウェアエンコーダを利用するかどうか
#   有効な値は 0, 1
#
# BOOST_ROOT: Boost のインストール先ディレクトリ
#
# WEBRTC_SRC_ROOT: WebRTC のソースディレクトリ
#
# WEBRTC_LIB_ROOT: WebRTC のビルドディレクトリ
#
# SYSROOT: TARGET_ARCH が arm の場合のクロスビルド用の RootFS ディレクトリ
#
# BUILD_MODE: ビルドモード。明示的に指定しない限り build になる。
#   有効な値は build, package
#
# BUILD_ROOT: ビルド用ディレクトリ。デフォルトでは ../momo-build になる。
#
# MOMO_CFLAGS: C コンパイラに追加で渡すフラグ。最適化フラグやデバッグフラグを入れることを想定している。
#
# MOMO_LDFLAGS: C リンカーに追加で渡すフラグ。サニタイズフラグや追加のリンクオブジェクトを入れることを想定している。

ifeq ($(PACKAGE_NAME),raspbian-stretch_armv6)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= raspbian-stretch
  TARGET_ARCH ?= arm
  TARGET_ARCH_ARM ?= armv6
  USE_ROS ?= 0
  USE_IL_ENCODER ?= 1
  BOOST_ROOT ?= /root/boost-$(BOOST_VERSION)
  WEBRTC_SRC_ROOT ?= /root/webrtc/src
  WEBRTC_LIB_ROOT ?= /root/webrtc-build/raspbian-stretch_armv6
  SYSROOT ?= /root/rootfs
else ifeq ($(PACKAGE_NAME),raspbian-stretch_armv7)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= raspbian-stretch
  TARGET_ARCH ?= arm
  TARGET_ARCH_ARM ?= armv7
  USE_ROS ?= 0
  USE_IL_ENCODER ?= 1
  BOOST_ROOT ?= /root/boost-$(BOOST_VERSION)
  WEBRTC_SRC_ROOT ?= /root/webrtc/src
  WEBRTC_LIB_ROOT ?= /root/webrtc-build/raspbian-stretch_armv7
  SYSROOT ?= /root/rootfs
else ifeq ($(PACKAGE_NAME),ubuntu-16.04_armv7_ros)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= ubuntu-16.04
  TARGET_ARCH ?= arm
  TARGET_ARCH_ARM ?= armv7
  USE_ROS ?= 1
  USE_IL_ENCODER ?= 1
  BOOST_ROOT ?= /root/boost-$(BOOST_VERSION)
  WEBRTC_SRC_ROOT ?= /root/webrtc/src
  WEBRTC_LIB_ROOT ?= /root/webrtc-build/ubuntu-16.04_armv7_ros
  SYSROOT ?= /root/rootfs
else ifeq ($(PACKAGE_NAME),ubuntu-16.04_armv8)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= ubuntu-16.04
  TARGET_ARCH ?= arm
  TARGET_ARCH_ARM ?= armv8
  USE_ROS ?= 0
  USE_IL_ENCODER ?= 0
  BOOST_ROOT ?= /root/boost-$(BOOST_VERSION)
  WEBRTC_SRC_ROOT ?= /root/webrtc/src
  WEBRTC_LIB_ROOT ?= /root/webrtc-build/ubuntu-16.04_armv8
  SYSROOT ?= /root/rootfs
else ifeq ($(PACKAGE_NAME),ubuntu-16.04_x86_64_ros)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= ubuntu-16.04
  TARGET_ARCH ?= x86_64
  USE_ROS ?= 1
  USE_IL_ENCODER ?= 0
  BOOST_ROOT ?= /root/boost-$(BOOST_VERSION)
  WEBRTC_SRC_ROOT ?= /root/webrtc/src
  WEBRTC_LIB_ROOT ?= /root/webrtc-build/ubuntu-16.04_x86_64_ros
else ifeq ($(PACKAGE_NAME),ubuntu-18.04_x86_64)
  TARGET_OS ?= linux
  TARGET_OS_LINUX ?= ubuntu-18.04
  TARGET_ARCH ?= x86_64
  USE_ROS ?= 0
  USE_IL_ENCODER ?= 0
  BOOST_ROOT ?= /root/boost-$(BOOST_VERSION)
  WEBRTC_SRC_ROOT ?= /root/webrtc/src
  WEBRTC_LIB_ROOT ?= /root/webrtc-build/ubuntu-18.04_x86_64
else ifeq ($(PACKAGE_NAME),macos)
  TARGET_OS ?= macos
  TARGET_ARCH ?= x86_64
  USE_ROS ?= 0
  USE_IL_ENCODER ?= 0
  BOOST_ROOT ?= $(CURDIR)/build/macos/boost-$(BOOST_VERSION)
  # CURDIR を付けると、ar に渡す時に引数が長すぎるって怒られたので、
  # 相対パスで指定する。
  WEBRTC_SRC_ROOT ?= build/macos/webrtc/src
  WEBRTC_LIB_ROOT ?= build/macos/webrtc-build/macos

  # depot_tools へのパスを通しておく
  # (Docker から実行するタイプのビルドでは事前に通してあるが、こっちは通してない可能性があるので)
  export PATH := $(CURDIR)/build/macos/webrtc/depot_tools:$(PATH)
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

  ifndef USE_IL_ENCODER
    $(info - USE_IL_ENCODER が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef BOOST_ROOT
    $(info - BOOST_ROOT が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef WEBRTC_SRC_ROOT
    $(info - WEBRTC_SRC_ROOT が指定されていません)
    HAS_ERROR = 1
  endif

  ifndef WEBRTC_LIB_ROOT
    $(info - WEBRTC_LIB_ROOT が指定されていません)
    HAS_ERROR = 1
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

CFLAGS += -Wno-macro-redefined -fno-lto -std=c++11 -pthread -DWEBRTC_POSIX -DOPENSSL_IS_BORINGSSL -Isrc/
CFLAGS += -I$(WEBRTC_SRC_ROOT) -I$(WEBRTC_SRC_ROOT)/third_party/libyuv/include -I$(WEBRTC_SRC_ROOT)/third_party/abseil-cpp
LDFLAGS += -L$(BUILD_ROOT) -lpthread

LDFLAGS += -lwebrtc

ifeq ($(USE_ROS),1)
  ifeq ($(TARGET_OS_LINUX),ubuntu-16.04)
    ROS_VERSION = kinetic
  else ifeq ($(TARGET_OS_LINUX),ubuntu-18.04)
    ROS_VERSION = melodic
  else
    $(error ROS_VERSION が決定できません)
  endif
endif

# x86_64/armv8 の場合は H264 を使わない
ifeq ($(TARGET_OS),linux)
  ifeq ($(TARGET_ARCH),x86_64)
    CFLAGS += -DUSE_H264=0
  else ifeq ($(TARGET_ARCH_ARM),armv8)
    CFLAGS += -DUSE_H264=0
  else
    CFLAGS += -DUSE_H264=1
  endif
else ifeq ($(TARGET_OS),macos)
  CFLAGS += -DUSE_H264=1
endif

ifeq ($(TARGET_OS),linux)
  CFLAGS += -fpic
  LDFLAGS += -lX11 -lXau -lXdmcp -lxcb -lplds4 -lXext -lexpat -ldl -lnss3 -lnssutil3 -lplc4 -lnspr4 -lrt

  CC = $(WEBRTC_SRC_ROOT)/third_party/llvm-build/Release+Asserts/bin/clang
  CXX = $(WEBRTC_SRC_ROOT)/third_party/llvm-build/Release+Asserts/bin/clang++
  AR = $(WEBRTC_SRC_ROOT)/third_party/llvm-build/Release+Asserts/bin/llvm-ar

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

    ifneq ($(TARGET_ARCH_ARM),armv8)
      # armv6, armv7 用

      CFLAGS += -mfloat-abi=hard
      ifeq ($(TARGET_ARCH_ARM),armv6)
        CFLAGS += -mcpu=arm1176jzf-s -mfpu=vfp
      else
        CFLAGS += -march=armv7-a -mtune=generic-armv7-a -mfpu=neon -mthumb
      endif

      # ilclient の設定
      ifeq ($(USE_IL_ENCODER),1)
        ifeq ($(TARGET_OS_LINUX),raspbian-stretch)
          VC_PATH = $(SYSROOT)/opt/vc
        else
          VC_PATH = $(SYSROOT)/usr
        endif

        IL_SOURCES = $(wildcard libs/ilclient/*.c)
        IL_OBJECTS = $(addprefix $(BUILD_ROOT)/,$(patsubst %.c,%.o,$(IL_SOURCES)))
        IL_CFLAGS = \
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
        IL_INCLUDES = \
          -I$(VC_PATH)/include/ \
          -I$(VC_PATH)/include/interface/vcos/pthreads \
          -I$(VC_PATH)/include/interface/vmcs_host/linux
        CFLAGS += \
          -DUSE_IL_ENCODER=1 \
          $(IL_CFLAGS) \
          -I$(VC_PATH)/include/ \
          -Ilibs/ilclient/ \
          -I.
        LDFLAGS += \
          -L$(VC_PATH)/lib/ \
          -lbrcmGLESv2 \
          -lbrcmEGL \
          -lopenmaxil \
          -lbcm_host \
          -lvcos \
          -lvchiq_arm \
          -lm
        SOURCES += $(shell find hwenc_il -maxdepth 1 -name '*.cpp')
        LDFLAGS += -lilclient
        LIBILCLIENT = $(BUILD_ROOT)/libilclient.a
      endif
    endif
  endif
endif

ifeq ($(TARGET_OS),macos)
  CC = $(WEBRTC_SRC_ROOT)/third_party/llvm-build/Release+Asserts/bin/clang
  CXX = $(WEBRTC_SRC_ROOT)/third_party/llvm-build/Release+Asserts/bin/clang++

  SDK_PATH = $(shell xcrun --sdk macosx --show-sdk-path)
  CC += --sysroot=$(SDK_PATH)
  CXX += --sysroot=$(SDK_PATH)

  CFLAGS += -DWEBRTC_POSIX -DWEBRTC_MAC
  CFLAGS += -fconstant-string-class=NSConstantString -I$(WEBRTC_SRC_ROOT)/sdk/objc -I$(WEBRTC_SRC_ROOT)/sdk/objc/base
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
  SOURCES += $(shell find src -name '*.mm')
else
  ifeq ($(USE_ROS),0)
    # USE_ROS=0 かつ mac 以外の場合はカスタムされた libc++ を使っているためオプション追加 
    CFLAGS += -D_LIBCPP_ABI_UNSTABLE
  endif
endif

ifeq ($(USE_ROS),1)
  # USE_ROS=1 の場合は libstdc++ を使う（追加オプション無し）
else
  # USE_ROS=0 の場合は libc++ を使う
  CFLAGS += -nostdinc++ -isystem$(WEBRTC_SRC_ROOT)/buildtools/third_party/libc++/trunk/include
endif

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
  SOURCES += $(shell find src -name '*.cpp')
else
  SOURCES += $(shell find src -type d -name 'ros' -prune -o -type f -name '*.cpp' -print)
endif

OBJECTS = $(addprefix $(BUILD_ROOT)/,$(patsubst %.mm,%.o,$(patsubst %.cpp,%.o,$(SOURCES))))

# Boost
CFLAGS += -I$(BOOST_ROOT)/include
LDFLAGS += -L$(BOOST_ROOT)/lib
# Boost.Beast で BoringSSL を使うので、そのあたりも追加する
CFLAGS += -I$(WEBRTC_SRC_ROOT)/third_party/boringssl/src/include -DOPENSSL_IS_BORINGSSL
LDFLAGS += -L$(WEBRTC_LIB_ROOT)/obj/third_party/boringssl -lboost_filesystem -lboringssl

# JSON
CFLAGS += -Ilibs/json-$(JSON_VERSION)/include

# CLI11
CFLAGS += -Ilibs/CLI11-$(CLI11_VERSION)/include

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

# libs/ilclient 以下のソースのビルドルール
$(BUILD_ROOT)/libs/ilclient/%.o: libs/ilclient/%.c | $(BUILD_ROOT)
	@mkdir -p `dirname $@`
	$(CC) $(IL_CFLAGS) $(IL_INCLUDES) -c $< -o $@ -Wno-deprecated-declarations

$(BUILD_ROOT)/libilclient.a: $(IL_OBJECTS) | $(BUILD_ROOT)
	$(AR) -rcT $@ $^

# WEBRTC_LIB_ROOT が空の時に find するとエラーになるので、
# WEBRTC_LIB_ROOT が定義されている時だけルールを定義する
ifdef WEBRTC_LIB_ROOT

$(BUILD_ROOT)/libwebrtc.a: $(shell find $(WEBRTC_LIB_ROOT)/obj -name '*.o') | $(BUILD_ROOT)
	@mkdir -p `dirname $@`
	$(AR) -r -c -s -D $@ $^

endif

# hwenc_il 以下のソースのビルドルール
$(BUILD_ROOT)/hwenc_il/%.o: hwenc_il/%.cpp | $(BUILD_ROOT)
	@mkdir -p `dirname $@`
	$(CXX) $(CFLAGS) $(INCLUDES) -c $< -o $@

# src 以下のソースのビルドルール
$(BUILD_ROOT)/src/%.o: src/%.cpp | $(BUILD_ROOT)
	@mkdir -p `dirname $@`
	$(CXX) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_ROOT)/src/%.o: src/%.mm | $(BUILD_ROOT)
	@mkdir -p `dirname $@`
	$(CXX) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_ROOT)/momo: $(LIBILCLIENT) $(BUILD_ROOT)/libwebrtc.a $(OBJECTS) | $(BUILD_ROOT)
	$(CXX) -o $(BUILD_ROOT)/momo $(OBJECTS) $(LDFLAGS)

.PHONY: momo
momo:
	# 依存ライブラリのビルドと取得
	cd $(WEBRTC_LIB_ROOT) && ninja
	if [ ! -e libs/json-$(JSON_VERSION)/include ]; then git clone --branch v$(JSON_VERSION) --depth 1 https://github.com/nlohmann/json.git libs/json-$(JSON_VERSION); fi
	if [ ! -e libs/CLI11-$(CLI11_VERSION)/include ]; then git clone --branch v$(CLI11_VERSION) --depth 1 https://github.com/CLIUtils/CLI11.git libs/CLI11-$(CLI11_VERSION); fi

	# momo 本体のビルド
	$(MAKE) $(BUILD_ROOT)/momo

	# ビルド後に ./momo p2p で動作確認できるようにしたいので、生成されたバイナリをルートディレクトリにコピーする
	cp $(BUILD_ROOT)/momo momo

.PHONY: clean
clean:
	rm -f $(shell find $(BUILD_ROOT) -type f -name '*.o')
	rm -f $(shell find $(BUILD_ROOT) -type f -name '*.a')
	rm -f $(shell find $(BUILD_ROOT) -type f -name '*.d')
	rm -f momo
