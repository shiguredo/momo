LIBWEBRTC = src/out/x86_64/obj/libwebrtc.a
BOOST_VERSION = 1.69.0
BOOST_PATH = boost_$(subst .,_,$(BOOST_VERSION))
BOOST = $(BOOST_PATH)/out

ifeq ($(JOBS),)
	JOBS := $(shell system_profiler SPHardwareDataType | grep "Total Number of Cores" |  awk '{split($$0,ary,":");print ary[2]}' | xargs)
	ifeq ($(JOBS),)
		JOBS := 1
	endif
endif

.PHONY: all
all: $(LIBWEBRTC) $(BOOST)
	make -C ../../ OUT_PATH=x86_64 RTC_ROOT=build/macos BOOST_PATH=build/macos/$(BOOST) MOMO_VERSION=$(MOMO_VERSION)

.PHONY: clean
clean:
	-rm -rf src/ > /dev/null 2>&1
	-rm -rf depot_tools/ > /dev/null 2>&1
	-rm .gclient > /dev/null 2>&1
	-rm .gclient_entries > /dev/null 2>&1
	-rm -rf $(BOOST_PATH)/ > /dev/null 2>&1

$(LIBWEBRTC):
	git clone https://chromium.googlesource.com/chromium/tools/depot_tools.git ./depot_tools
	export PATH="$(shell pwd)/depot_tools:$(PATH)" && \
	export GYP_DEFINES="OS=mac target_arch=x64" && \
	fetch webrtc && \
	cd src && \
	git fetch && \
	git checkout -b m$(WEBRTC_VERSION) remotes/branch-heads/$(WEBRTC_VERSION) && \
	gclient sync && \
	gn gen out/x86_64 --args='target_os="mac" is_debug=false rtc_include_tests=false rtc_build_examples=false rtc_use_h264=true is_component_build=false use_rtti=true' && \
	ninja -C out/x86_64 && \
	ninja -C out/x86_64 \
	  builtin_audio_decoder_factory \
	  builtin_video_decoder_factory \
	  builtin_video_encoder_factory \
	  peerconnection \
	  create_pc_factory

$(BOOST):
	curl -LO https://dl.bintray.com/boostorg/release/$(BOOST_VERSION)/source/$(BOOST_PATH).tar.gz && \
	tar xf $(BOOST_PATH).tar.gz && \
	rm $(BOOST_PATH).tar.gz && \
	cd $(BOOST_PATH) && \
	./bootstrap.sh && \
	./b2 link=static variant=release install -j$(JOBS) --prefix=out --with-system --with-filesystem
