include ../../VERSION

LIBWEBRTC = $(CURDIR)/webrtc-build/obj/libwebrtc.a
BOOST = $(CURDIR)/boost-$(BOOST_VERSION)

ifeq ($(JOBS),)
	JOBS := $(shell system_profiler SPHardwareDataType | grep "Total Number of Cores" |  awk '{split($$0,ary,":");print ary[2]}' | xargs)
	ifeq ($(JOBS),)
		JOBS := 1
	endif
endif

.PHONY: all
all: $(LIBWEBRTC) $(BOOST)

.PHONY: clean
clean:
	rm -rf webrtc
	rm -rf webrtc-build
	rm -rf boost
	rm -rf boost-*

.PHONY: check
check:
	@if [ "`python --version 2>&1 | cut -d' ' -f2 | cut -b1-3`" != "2.7" ]; then \
	  echo ""; \
	  echo "Python 3 では WebRTC をビルドできません。Python 2.7 を利用して下さい。"; \
	  echo "macOS でのビルド方法については doc/BUILD.md を確認して下さい。"; \
	  echo ""; \
	  exit 1; \
	fi

$(LIBWEBRTC): check
	source ../../script/docker.sh && \
	export PATH="$(CURDIR)/webrtc/depot_tools:$(PATH)" && \
	export GYP_DEFINES="OS=mac target_arch=x64" && \
	prepare_webrtc_macos "" $(WEBRTC_COMMIT) $(CURDIR)/webrtc && \
	cd $(CURDIR)/webrtc/src && \
	patch -p2 < $(CURDIR)/../../patch/4k.patch && \
	gn gen $(CURDIR)/webrtc-build/macos --args='target_os="mac" is_debug=false rtc_include_tests=false rtc_build_examples=false rtc_use_h264=false is_component_build=false use_rtti=true libcxx_abi_unstable=false' && \
	ninja -C $(CURDIR)/webrtc-build/macos && \
	ninja -C $(CURDIR)/webrtc-build/macos \
	  builtin_audio_decoder_factory \
	  native_api \
	  default_codec_factory_objc \
	  peerconnection \
	  videocapture_objc

$(BOOST):
	source ../../script/docker.sh && \
	setup_boost $(BOOST_VERSION) $(CURDIR)/boost && \
	cd $(CURDIR)/boost/source && \
	./b2 visibility=hidden link=static variant=release install -j$(JOBS) --prefix=$(CURDIR)/boost-$(BOOST_VERSION) --ignore-site-config --with-filesystem
