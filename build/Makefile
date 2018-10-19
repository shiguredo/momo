BUILD_NUMBER=$(shell date +"%Y%m%d%H%M%S")
WEBRTC_VERSION=71

TARGET=$@

.PHONY: all
all: armv6 armv7 armv8

.PHONY: armv6 armv7 armv8
armv6 armv7 armv8:
	# ARM 系ビルド全体で共通のビルド用イメージを作る
	docker image build --rm -t momo/webrtc-arm-buildbase arm --file arm/buildbase.Dockerfile

	# 各 ARM 用ビルド
	$(MAKE) -C arm/$(TARGET) WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER)

	# 生成されたバイナリをこのディレクトリに移動
	mv arm/$(TARGET)/momo ./momo-$(TARGET)-m$(WEBRTC_VERSION)-$(BUILD_NUMBER)

# WebRTC から再ビルド。
# 同じ WEBRTC_VERSION の WebRTC を再取得してビルドしたい時に使う。
.PHONY: armv6.rebuild
armv6.rebuild:
	$(MAKE) -C arm/armv6 clean.image WEBRTC_VERSION=$(WEBRTC_VERSION)
	$(MAKE) -C arm/armv6 WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER)

.PHONY: armv7.rebuild
armv7.rebuild:
	$(MAKE) -C arm/armv7 clean.image WEBRTC_VERSION=$(WEBRTC_VERSION)
	$(MAKE) -C arm/armv7 WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER)

.PHONY: armv8.rebuild
armv8.rebuild:
	$(MAKE) -C arm/armv8 clean.image WEBRTC_VERSION=$(WEBRTC_VERSION)
	$(MAKE) -C arm/armv8 WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER)

# 各プラットフォーム用イメージの掃除。これをすると全体的にビルドし直すことになる。
.PHONY: armv6.clean
armv6.clean:
	$(MAKE) -C arm/armv6 clean.image.all WEBRTC_VERSION=$(WEBRTC_VERSION)

.PHONY: armv7.clean
armv7.clean:
	$(MAKE) -C arm/armv7 clean.image.all WEBRTC_VERSION=$(WEBRTC_VERSION)

.PHONY: armv8.clean
armv8.clean:
	$(MAKE) -C arm/armv8 clean.image.all WEBRTC_VERSION=$(WEBRTC_VERSION)

.PHONY: x86_64
x86_64:
	# x86_64 用ビルド
	$(MAKE) -C $(TARGET) WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER)

	# 生成されたバイナリをこのディレクトリに移動
	mv $(TARGET)/momo ./momo-$(TARGET)-m$(WEBRTC_VERSION)-$(BUILD_NUMBER)

.PHONY: macos
macos:
	$(MAKE) -C $(TARGET) WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER)
	mv ../momo momo-$(TARGET)-m$(WEBRTC_VERSION)-$(BUILD_NUMBER)

.PHONY: macos.clean
macos.clean:
	$(MAKE) -C macos clean WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER)

.PHONY: pkg
MOMO_VERSION=18.10.0-rc2
FILENAMES:=LICENSE NOTICE html

define COPY
	cp -r $(1) $(2)

endef

pkg: armv6.pkg armv7.pkg armv8.pkg x86_64.pkg

armv6.pkg:
	mkdir momo-$(MOMO_VERSION)_raspbian_armv6
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_raspbian_armv6/))
	cp $(lastword $(sort $(wildcard ./momo-armv6-*))) momo-$(MOMO_VERSION)_raspbian_armv6/momo
	tar czf momo-$(MOMO_VERSION)_raspbian_armv6.tar.gz momo-$(MOMO_VERSION)_raspbian_armv6
	rm -rf momo-$(MOMO_VERSION)_raspbian_armv6

armv7.pkg:
	mkdir momo-$(MOMO_VERSION)_raspbian_armv7
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_raspbian_armv7/))
	cp $(lastword $(sort $(wildcard ./momo-armv7-*))) momo-$(MOMO_VERSION)_raspbian_armv7/momo
	tar czf momo-$(MOMO_VERSION)_raspbian_armv7.tar.gz momo-$(MOMO_VERSION)_raspbian_armv7
	rm -rf momo-$(MOMO_VERSION)_raspbian_armv7

armv8.pkg:
	mkdir momo-$(MOMO_VERSION)_ubuntu_armv8
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_ubuntu_armv8/))
	cp $(lastword $(sort $(wildcard ./momo-armv8-*))) momo-$(MOMO_VERSION)_ubuntu_armv8/momo
	tar czf momo-$(MOMO_VERSION)_ubuntu_armv8.tar.gz momo-$(MOMO_VERSION)_ubuntu_armv8
	rm -rf momo-$(MOMO_VERSION)_ubuntu_armv8

x86_64.pkg:
	mkdir momo-$(MOMO_VERSION)_ubuntu_x86_64
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_ubuntu_x86_64/))
	cp $(lastword $(sort $(wildcard ./momo-x86_64-*))) momo-$(MOMO_VERSION)_ubuntu_x86_64/momo
	tar czf momo-$(MOMO_VERSION)_ubuntu_x86_64.tar.gz momo-$(MOMO_VERSION)_ubuntu_x86_64
	rm -rf momo-$(MOMO_VERSION)_ubuntu_x86_64

clean.pkg:
	rm -rf momo-$(MOMO_VERSION)_*
