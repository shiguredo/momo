ifndef RTC_ROOT
	RTC_ROOT = $(HOME)
endif

ifndef BOOST_PATH
	BOOST_PATH = /root/boost-1.69.0
endif

ifndef ROS_VERSION
	USE_WEBRTC_LIBCXX = 1
endif

UNAME := $(shell uname -s)

TARGET = momo

RTC_LIB = libWebRTC_$(OUT_PATH).a

RTC_LIB_PATH=$(RTC_ROOT)/src/out/$(OUT_PATH)

CFLAGS += -Wno-macro-redefined -fno-lto -pthread -std=c++11 -DWEBRTC_POSIX -DOPENSSL_IS_BORINGSSL -Isrc/

ifdef USE_WEBRTC_LIBCXX
	CFLAGS += -nostdinc++ -isystem$(RTC_ROOT)/src/buildtools/third_party/libc++/trunk/include
endif

ifdef MOMO_VERSION
	CFLAGS += -DMOMO_VERSION='"$(MOMO_VERSION)"'
endif

# webrtc
CFLAGS += -I$(RTC_ROOT)/src -I$(RTC_ROOT)/src/third_party/libyuv/include -I$(RTC_ROOT)/src/third_party/abseil-cpp

# H264 を使うかどうかの設定。
# x86_64/armv8 の場合は H264 を使わない
ifeq ($(UNAME),Linux)
	ifneq (,$(findstring arm64,$(OUT_PATH)))
		CFLAGS += -DUSE_H264=0
	else
		ifneq (,$(findstring x86_64,$(OUT_PATH)))
			CFLAGS += -DUSE_H264=0
		else
			CFLAGS += -DUSE_H264=1
		endif
	endif
endif

ifeq ($(UNAME),Linux)
	CFLAGS += -fpic
	LDFLAGS += -lX11 -lXau -lXdmcp -lxcb -lplds4 -lXext -lexpat -ldl -lnss3 -lnssutil3 -lplc4 -lnspr4 -lrt
	CFLAGS += -I$(BOOST_PATH)/include
	LDFLAGS += -L$(BOOST_PATH)/lib

	ifneq (,$(findstring arm,$(OUT_PATH)))
		SYSROOT = $(RTC_ROOT)/rootfs
		ifneq (,$(findstring arm64,$(OUT_PATH)))
			TARGET_ARCH = aarch64-linux-gnu
		else
			TARGET_ARCH = arm-linux-gnueabihf
			CFLAGS += -mfloat-abi=hard
			ifneq (,$(findstring armv6,$(OUT_PATH)))
				CFLAGS += -mcpu=arm1176jzf-s -mfpu=vfp
			else
				CFLAGS += -march=armv7-a -mtune=generic-armv7-a -mfpu=neon -mthumb
			endif
			USE_IL_ENCODER ?= 1
			# ilclient
			ifneq ($(USE_IL_ENCODER),0)
				ifneq (,$(findstring raspbian,$(OUT_PATH)))
					VC_PATH = $(SYSROOT)/opt/vc
				else
					VC_PATH = $(SYSROOT)/usr
				endif
				ILCLIENT_DIR = libs/ilclient
				IL_SOURCE = $(shell find $(ILCLIENT_DIR) -name '*.c')
				IL_OBJECT = $(IL_SOURCE:%.c=%.o)
				ILCLIENT_LIB = libilclient.a
				IL_CFLAGS = -DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -fPIC -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -g -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM
				IL_INCLUDES = -I$(VC_PATH)/include/ -I$(VC_PATH)/include/interface/vcos/pthreads -I$(VC_PATH)/include/interface/vmcs_host/linux
				CFLAGS += -DUSE_IL_ENCODER=1 $(IL_CFLAGS) -I$(VC_PATH)/include/ -I$(ILCLIENT_DIR)/ -I.
				LDFLAGS += -L$(VC_PATH)/lib/ -lbrcmGLESv2 -lbrcmEGL -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lm
				SOURCE += $(shell find $(CURDIR)/hwenc_il -maxdepth 1 -name '*.cpp') $(ILCLIENT_LIB)
			endif
		endif
	endif

	CC = $(RTC_ROOT)/src/third_party/llvm-build/Release+Asserts/bin/clang
	CXX = $(RTC_ROOT)/src/third_party/llvm-build/Release+Asserts/bin/clang++
	AR = $(RTC_ROOT)/src/third_party/llvm-build/Release+Asserts/bin/llvm-ar
	ifdef TARGET_ARCH
		CC += --sysroot=$(SYSROOT) --target=$(TARGET_ARCH)
		CXX += --sysroot=$(SYSROOT) --target=$(TARGET_ARCH)
		ifdef USE_WEBRTC_LIBCXX
			CFLAGS += -I$(SYSROOT)/usr/include/$(TARGET_ARCH)
			LDFLAGS += -L$(SYSROOT)/usr/lib/$(TARGET_ARCH)
		endif
	endif
endif

ifeq ($(UNAME),Darwin)
	CFLAGS += -DWEBRTC_POSIX -DWEBRTC_MAC
	LDFLAGS += -F/System/Library/Frameworks -ldl -framework Foundation -framework AVFoundation -framework CoreServices -framework CoreFoundation -framework AudioUnit -framework AudioToolbox -framework CoreAudio -framework CoreGraphics -framework CoreMedia -framework CoreVideo
	LIBRTCCONDUCTOR = lib$(RTCCONDUCTOR)_mac.so
endif

ifdef ROS_VERSION
	CFLAGS += -DHAVE_JPEG=1 -DUSE_ROS=1 -I$(SYSROOT)/opt/ros/$(ROS_VERSION)/include
	LDFLAGS += -lpthread -L$(SYSROOT)/opt/ros/$(ROS_VERSION)/lib -lmessage_filters -lroscpp -lrosconsole -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lrosconsole_backend_interface
	SOURCE += $(shell find $(CURDIR)/src -name '*.cpp')
else
	SOURCE += $(shell find $(CURDIR)/src -type d -name 'ros' -prune -o -type f -name '*.cpp' -print)
endif
SOURCE += $(RTC_LIB)

# boost
CFLAGS += -I$(RTC_ROOT)/src/third_party/boringssl/src/include -DOPENSSL_IS_BORINGSSL
LDFLAGS += -L$(RTC_LIB_PATH)/obj/third_party/boringssl -lboost_system -lboost_filesystem -lboringssl

# json
CFLAGS += -Ilibs/json/include

# CLI11
CFLAGS += -Ilibs/CLI11/include

# package
ifeq ($(BUILD_MODE),pkg)
	LDFLAGS += -s
endif

all: git.init $(TARGET)

.PHONY: git.init
git.init:
	git submodule update -i

$(IL_OBJECT): $(ILCLIENT_DIR)/%.o : $(ILCLIENT_DIR)/%.c
	@rm -f $@ 
	$(CC) $(IL_CFLAGS) $(IL_INCLUDES) -g -c $< -o $@ -Wno-deprecated-declarations

$(ILCLIENT_LIB): $(IL_OBJECT)
	$(AR) -rcT $@ $^

$(RTC_LIB): $(shell find $(RTC_LIB_PATH)/obj -name '*.o')
	$(AR) -rcT $@ $^

$(TARGET): $(SOURCE)
	$(CXX) -o $@ $(CFLAGS) $(INCLUDES) $^ $(LDFLAGS)

clean:
	rm -f *.o *.a $(TARGET)
