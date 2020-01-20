include ../../VERSION

WEBRTC_PREFIX = $(CURDIR)/webrtc
BOOST_PREFIX = $(CURDIR)/boost
SDL2_PREFIX = $(CURDIR)/sdl2

WEBRTC = $(WEBRTC_PREFIX)/lib/libwebrtc.a
BOOST = $(BOOST_PREFIX)/include/boost/version.hpp
SDL2 = $(SDL2_PREFIX)/lib/libSDL2.a
LLVM = $(CURDIR)/llvm/clang/bin/clang++
CLI11 = $(CURDIR)/CLI11/include/CLI/Version.hpp
JSON = $(CURDIR)/json/include/nlohmann/json.hpp

ifeq ($(JOBS),)
	JOBS := $(shell system_profiler SPHardwareDataType | grep "Total Number of Cores" |  awk '{split($$0,ary,":");print ary[2]}' | xargs)
	ifeq ($(JOBS),)
		JOBS := 1
	endif
endif

.PHONY: all
all: $(WEBRTC) $(BOOST) $(SDL2) $(LLVM) $(CLI11) $(JSON)

.PHONY: clean
clean:
	rm -rf webrtc
	rm -rf boost-source
	rm -rf boost
	rm -rf sdl2-source
	rm -rf sdl2
	rm -rf llvm

$(CLI11): ../../VERSION
	rm -rf CLI11
	git clone --branch v$(CLI11_VERSION) --depth 1 https://github.com/CLIUtils/CLI11.git

$(JSON): ../../VERSION
	rm -rf json
	git clone --branch v$(JSON_VERSION) --depth 1 https://github.com/nlohmann/json.git

$(LLVM): $(WEBRTC)
	../../script/get_llvm.sh $(WEBRTC_PREFIX) $(CURDIR)
	touch $(LLVM)

$(WEBRTC): ../../VERSION
	../../script/get_webrtc.sh $(WEBRTC_BUILD_VERSION) macos $(CURDIR)
	touch $(WEBRTC)

$(BOOST): ../../VERSION
	../../script/setup_boost.sh $(BOOST_VERSION) $(CURDIR)/boost-source
	cd $(CURDIR)/boost-source/source && ./b2 visibility=hidden link=static variant=release install -j$(JOBS) --prefix=$(BOOST_PREFIX) --ignore-site-config --with-filesystem
	touch $(BOOST)

$(SDL2): $(LLVM)
	../../script/setup_sdl2.sh $(SDL2_VERSION) $(CURDIR)/sdl2-source
	rm -rf $(CURDIR)/sdl2-source/build
	mkdir -p $(CURDIR)/sdl2-source/build
	cd $(CURDIR)/sdl2-source/build && ../source/configure --disable-shared --prefix=$(SDL2_PREFIX)
	cd $(CURDIR)/sdl2-source/build && make && make install
	touch $(SDL2)
