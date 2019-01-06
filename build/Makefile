BUILD_NUMBER=$(shell date +"%Y%m%d%H%M%S")
WEBRTC_VERSION=71
MOMO_VERSION=18.12.0

RASPBIAN_VERSION=stretch

TARGET=$@

.PHONY: all
all: armv6 armv7 armv8

.PHONY: armv6 armv7 armv8 armv7_ros
armv6 armv7 armv8 armv7_ros:
	# ARM 系ビルド全体で共通のビルド用イメージを作る
	docker image build --rm -t momo/webrtc-arm-buildbase arm --file arm/buildbase.Dockerfile

	# 各 ARM 用ビルド
	$(MAKE) -C arm/$(TARGET) WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER) MOMO_VERSION=$(MOMO_VERSION)

	# 生成されたバイナリをこのディレクトリに移動
	mv arm/$(TARGET)/momo ./momo-$(TARGET)-m$(WEBRTC_VERSION)-$(BUILD_NUMBER)

# WebRTC から再ビルド。
# 同じ WEBRTC_VERSION の WebRTC を再取得してビルドしたい時に使う。
.PHONY: armv6.rebuild
armv6.rebuild:
	$(MAKE) -C arm/armv6 clean.image WEBRTC_VERSION=$(WEBRTC_VERSION)
	$(MAKE) -C arm/armv6 WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER) MOMO_VERSION=$(MOMO_VERSION)

.PHONY: armv7.rebuild
armv7.rebuild:
	$(MAKE) -C arm/armv7 clean.image WEBRTC_VERSION=$(WEBRTC_VERSION)
	$(MAKE) -C arm/armv7 WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER) MOMO_VERSION=$(MOMO_VERSION)

.PHONY: armv8.rebuild
armv8.rebuild:
	$(MAKE) -C arm/armv8 clean.image WEBRTC_VERSION=$(WEBRTC_VERSION)
	$(MAKE) -C arm/armv8 WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER) MOMO_VERSION=$(MOMO_VERSION)

.PHONY: armv7_ros.rebuild
armv7_ros.rebuild:
	$(MAKE) -C arm/armv7_ros clean.image WEBRTC_VERSION=$(WEBRTC_VERSION)
	$(MAKE) -C arm/armv7_ros WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER) MOMO_VERSION=$(MOMO_VERSION)

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

.PHONY: armv7_ros.clean
armv7_ros.clean:
	$(MAKE) -C arm/armv7_ros clean.image.all WEBRTC_VERSION=$(WEBRTC_VERSION)

.PHONY: ubuntu-1804_x86_64
ubuntu-1804_x86_64:
	# x86_64 用ビルド
	$(MAKE) -C x86_64 WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER) MOMO_VERSION=$(MOMO_VERSION)

	# 生成されたバイナリをこのディレクトリに移動
	mv x86_64/momo ./momo-ubuntu-18.04_x86_64-m$(WEBRTC_VERSION)-$(BUILD_NUMBER)

.PHONY: ubuntu-1604_x86_64_ros
ubuntu-1604_x86_64_ros:
	# x86_64_ros 用ビルド
	$(MAKE) -C x86_64_ros WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER) MOMO_VERSION=$(MOMO_VERSION)
	mv x86_64_ros/momo ./momo-ubuntu-16.04_x86_64_ros-m$(WEBRTC_VERSION)-$(BUILD_NUMBER)

.PHONY: macos
macos:
	$(MAKE) -C $(TARGET) WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER) MOMO_VERSION=$(MOMO_VERSION)
	mv ../momo momo-$(TARGET)-m$(WEBRTC_VERSION)-$(BUILD_NUMBER)

.PHONY: macos.clean
macos.clean:
	$(MAKE) -C macos clean WEBRTC_VERSION=$(WEBRTC_VERSION) BUILD_NUMBER=$(BUILD_NUMBER) MOMO_VERSION=$(MOMO_VERSION)

.PHONY: pkg
FILENAMES:=LICENSE NOTICE html

define COPY
	cp -r $(1) $(2)

endef

pkg: armv6.pkg armv7.pkg armv8.pkg ubuntu-1804_x86_64.pkg ubuntu-1804_x86_64_ros.pkg armv7_ros.pkg

armv6.pkg:
	mkdir momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv6
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv6/))
	cp $(lastword $(sort $(wildcard ./momo-armv6-*))) momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv6/momo
	tar czf momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv6.tar.gz momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv6
	rm -rf momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv6

armv7.pkg:
	mkdir momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv7
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv7/))
	cp $(lastword $(sort $(wildcard ./momo-armv7-*))) momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv7/momo
	tar czf momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv7.tar.gz momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv7
	rm -rf momo-$(MOMO_VERSION)_raspbian-$(RASPBIAN_VERSION)_armv7

armv8.pkg:
	mkdir momo-$(MOMO_VERSION)_ubuntu-16.04_armv8
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_ubuntu-16.04_armv8/))
	cp $(lastword $(sort $(wildcard ./momo-armv8-*))) momo-$(MOMO_VERSION)_ubuntu-16.04_armv8/momo
	tar czf momo-$(MOMO_VERSION)_ubuntu-16.04_armv8.tar.gz momo-$(MOMO_VERSION)_ubuntu-16.04_armv8
	rm -rf momo-$(MOMO_VERSION)_ubuntu-16.04_armv8

ubuntu-1804_x86_64.pkg:
	mkdir momo-$(MOMO_VERSION)_ubuntu-18.04_x86_64
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_ubuntu-18.04_x86_64/))
	cp $(lastword $(sort $(wildcard ./momo-ubuntu-18.04_x86_64-*))) momo-$(MOMO_VERSION)_ubuntu-18.04_x86_64/momo
	tar czf momo-$(MOMO_VERSION)_ubuntu-18.04_x86_64.tar.gz momo-$(MOMO_VERSION)_ubuntu-18.04_x86_64
	rm -rf momo-$(MOMO_VERSION)_ubuntu-18.04_x86_64

ubuntu-1604_x86_64_ros.pkg:
	mkdir momo-$(MOMO_VERSION)_ubuntu-16.04_x86_64_ros
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_ubuntu-16.04_x86_64_ros/))
	cp $(lastword $(sort $(wildcard ./momo-ubuntu-16.04_x86_64_ros-*))) momo-$(MOMO_VERSION)_ubuntu-16.04_x86_64_ros/momo
	tar czf momo-$(MOMO_VERSION)_ubuntu-16.04_x86_64_ros.tar.gz momo-$(MOMO_VERSION)_ubuntu-16.04_x86_64_ros
	rm -rf momo-$(MOMO_VERSION)_ubuntu-16.04_x86_64_ros

armv7_ros.pkg:
	mkdir momo-$(MOMO_VERSION)_ubuntu-16.04_armv7_ros
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_ubuntu-16.04_armv7_ros/))
	cp $(lastword $(sort $(wildcard ./momo-armv7_ros-*))) momo-$(MOMO_VERSION)_ubuntu-16.04_armv7_ros/momo
	tar czf momo-$(MOMO_VERSION)_ubuntu-16.04_armv7_ros.tar.gz momo-$(MOMO_VERSION)_ubuntu-16.04_armv7_ros
	rm -rf momo-$(MOMO_VERSION)_ubuntu-16.04_armv7_ros

macos.pkg:
	mkdir momo-$(MOMO_VERSION)_macos
	$(foreach filename,$(FILENAMES),$(call COPY,../$(filename),momo-$(MOMO_VERSION)_macos/))
	cp $(lastword $(sort $(wildcard ./momo-macos-*))) momo-$(MOMO_VERSION)_macos/momo
	tar czf momo-$(MOMO_VERSION)_macos.tar.gz momo-$(MOMO_VERSION)_macos
	rm -rf momo-$(MOMO_VERSION)_macos

clean.pkg:
	rm -rf momo-$(MOMO_VERSION)_*
