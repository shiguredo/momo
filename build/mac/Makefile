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

$(LIBWEBRTC):
	source ../../script/docker.sh && \
	export PATH="$(CURDIR)/webrtc/depot_tools:$(PATH)" && \
	export GYP_DEFINES="OS=mac target_arch=x64" && \
	prepare_webrtc_mac "" $(WEBRTC_COMMIT) $(CURDIR)/webrtc && \
	cd $(CURDIR)/webrtc/src && \
	patch -p2 < $(CURDIR)/../../patch/4k.patch && \
	gn gen $(CURDIR)/webrtc-build/mac --args='target_os="mac" is_debug=false rtc_include_tests=false rtc_build_examples=false rtc_use_h264=true is_component_build=false use_rtti=true' && \
	ninja -C $(CURDIR)/webrtc-build/mac && \
	ninja -C $(CURDIR)/webrtc-build/mac \
	  builtin_audio_decoder_factory \
	  native_api \
	  default_codec_factory_objc \
	  peerconnection \
	  create_pc_factory

$(BOOST):
	source ../../script/docker.sh && \
	setup_boost $(BOOST_VERSION) $(CURDIR)/boost && \
	cd $(CURDIR)/boost/source && \
	./b2 visibility=global link=static variant=release install -j$(JOBS) --prefix=$(CURDIR)/boost-$(BOOST_VERSION) --with-filesystem
