ifndef ARCH
	ARCH=arm
endif

ifndef OS
	OS=raspbian
endif

ifndef RTC_ROOT
	RTC_ROOT=$(HOME)
endif

UNAME := $(shell uname -s)

TARGET = momo

RTC_LIB = libWebRTC_$(ARCH).a

CFLAGS += -fno-lto -pthread -std=gnu++11 -nostdinc++ -isystem$(RTC_ROOT)/src/buildtools/third_party/libc++/trunk/include -DWEBRTC_POSIX -DOPENSSL_IS_BORINGSSL -Isrc/
ifdef MOMO_VERSION
	CFLAGS += -DMOMO_VERSION='"$(MOMO_VERSION)"'
endif

# webrtc
CFLAGS += -I$(RTC_ROOT)/src -I$(RTC_ROOT)/src/third_party/libyuv/include -I$(RTC_ROOT)/src/third_party/abseil-cpp

ifeq ($(UNAME),Linux)
	CC = $(RTC_ROOT)/src/third_party/llvm-build/Release+Asserts/bin/clang
	CXX = $(RTC_ROOT)/src/third_party/llvm-build/Release+Asserts/bin/clang++
	AR = $(RTC_ROOT)/src/third_party/llvm-build/Release+Asserts/bin/llvm-ar
	CIVETWEB_OPT += -DUSE_WEBSOCKET -nostdinc++ -isystem$(RTC_ROOT)/src/buildtools/third_party/libc++/trunk/include
	CFLAGS += -fpic
	LDFLAGS += -lX11 -lXau -lXdmcp -lxcb -lplds4 -lXext -lexpat -ldl -lnss3 -lnssutil3 -lplc4 -lnspr4 -lrt
	CFLAGS += -I/root/boost-1.68.0/include
	LDFLAGS += -L/root/boost-1.68.0/lib
	ifneq (,$(findstring arm,$(ARCH)))
		ifneq (,$(findstring arm64,$(ARCH)))
			CC += --sysroot=$(SYSROOT) --target=aarch64-linux-gnu
			CXX += --sysroot=$(SYSROOT) --target=aarch64-linux-gnu
			CIVETWEB_OPT += -I$(SYSROOT)/usr/include/aarch64-linux-gnu
			CFLAGS += -I$(SYSROOT)/usr/include/aarch64-linux-gnu
			LDFLAGS += -L$(SYSROOT)/usr/lib/aarch64-linux-gnu
		else
			USE_IL_ENCODER ?= 1
			CC += --sysroot=$(SYSROOT) --target=arm-linux-gnueabihf
			CXX += --sysroot=$(SYSROOT) --target=arm-linux-gnueabihf
			CIVETWEB_OPT += -I$(SYSROOT)/usr/include/arm-linux-gnueabihf
			CFLAGS += -I$(SYSROOT)/usr/include/arm-linux-gnueabihf
			LDFLAGS += -L$(SYSROOT)/usr/lib/arm-linux-gnueabihf -B$(SYSROOT)/usr/lib/arm-linux-gnueabihf
			# ilclient
			ifneq ($(USE_IL_ENCODER),0)
				ILCLIENT_DIR = libs/ilclient
				IL_SOURCE = $(shell find $(ILCLIENT_DIR) -name '*.c')
				IL_OBJECT = $(IL_SOURCE:%.c=%.o)
				ILCLIENT_LIB = libilclient.a
				IL_CFLAGS = -DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -fPIC -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -g -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM
				IL_INCLUDES = -I$(SYSROOT)/opt/vc/include/ -I$(SYSROOT)/opt/vc/include/interface/vcos/pthreads -I$(SYSROOT)/opt/vc/include/interface/vmcs_host/linux
				CFLAGS += -DUSE_IL_ENCODER=1 $(IL_CFLAGS) -I$(SYSROOT)/opt/vc/include/ -I$(ILCLIENT_DIR)/ -I.
				LDFLAGS += -mcpu=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard -L$(SYSROOT)/opt/vc/lib/ -lbrcmGLESv2 -lbrcmEGL -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lm
				SOURCE += $(shell find $(CURDIR)/hwenc_il -maxdepth 1 -name '*.cpp') $(ILCLIENT_LIB)
			else
			endif
		endif
		SYSROOT = $(RTC_ROOT)/rootfs
		RTC_LIB_PATH=$(RTC_ROOT)/src/out/$(ARCH)-$(OS)
	else
		RTC_LIB_PATH=$(RTC_ROOT)/src/out/$(ARCH)
	endif
endif

ifeq ($(UNAME),Darwin)
	CFLAGS += -DWEBRTC_POSIX -DWEBRTC_MAC
	LDFLAGS += -F/System/Library/Frameworks -ldl -framework Foundation -framework AVFoundation -framework CoreServices -framework CoreFoundation -framework AudioUnit -framework AudioToolbox -framework CoreAudio -framework CoreGraphics -framework CoreMedia -framework CoreVideo
	LIBRTCCONDUCTOR = lib$(RTCCONDUCTOR)_mac.so
	RTC_LIB_PATH=$(RTC_ROOT)/src/out/$(ARCH)
endif

SOURCE += $(shell find $(CURDIR)/src -name '*.cpp') $(RTC_LIB)

# boost
LDFLAGS += -lboost_system -lboost_filesystem

# json
CFLAGS += -Ilibs/json/include

# websocketpp
WEBSOCKETPP_DIR = libs/websocketpp
CFLAGS += -I$(WEBSOCKETPP_DIR)/ -I$(RTC_ROOT)/src/third_party/boringssl/src/include -D_WEBSOCKETPP_CPP11_STL_ -DOPENSSL_IS_BORINGSSL
LDFLAGS += -L$(RTC_LIB_PATH)/obj/third_party/boringssl -lboost_system -lboost_filesystem -lboringssl

# civetweb
CIVETWEB_DIR = libs/civetweb
CIVETWEB_LIB = libcivetweb.a
CFLAGS += -I$(CIVETWEB_DIR)/include
SOURCE += $(CIVETWEB_LIB)

# CLI11
CFLAGS += -Ilibs/CLI11/include

# package
ifeq ($(BUILD_MODE),pkg)
	LDFLAGS += -s
endif

all: $(TARGET)

$(IL_OBJECT): $(ILCLIENT_DIR)/%.o : $(ILCLIENT_DIR)/%.c
	@rm -f $@ 
	$(CC) $(IL_CFLAGS) $(IL_INCLUDES) -g -c $< -o $@ -Wno-deprecated-declarations

$(ILCLIENT_LIB): $(IL_OBJECT)
	$(AR) -rcT $@ $^

$(RTC_LIB): $(shell find $(RTC_LIB_PATH)/obj -name '*.o')
	$(AR) -rcT $@ $^

$(CIVETWEB_LIB):
	git submodule init
	git submodule update
	cd $(CIVETWEB_DIR) && git reset --hard
	patch -p1 $(CIVETWEB_DIR)/src/civetweb.c < patch/civetweb_signal.patch
	$(MAKE) -C $(CIVETWEB_DIR) clean lib WITH_CPP=1 WITH_WEBSOCKET=1 CC="$(CC)" CXX="$(CXX)" COPT="$(CIVETWEB_OPT)"
	cp $(CIVETWEB_DIR)/$(CIVETWEB_LIB) .

$(TARGET): $(SOURCE)
	$(CXX) -o $@ $(CFLAGS) $(INCLUDES) $^ $(LDFLAGS)

clean:
	rm -f *.o *.a $(TARGET)
