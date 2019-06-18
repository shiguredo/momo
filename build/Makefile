include ../VERSION

.PHONY: help
help:
	@echo "各環境用の momo バイナリやパッケージを作るための Makefile です。"
	@echo ""
	@echo "使い方:"
	@echo "  make [オプション] <パッケージ名>"
	@echo "  make [オプション] <パッケージ名>.package"
	@echo "  make [オプション] <パッケージ名>.edit"
	@echo "  make [オプション] <パッケージ名>.clean"
	@echo ""
	@echo "<パッケージ名>:"
	@for pkg in $(PACKAGE_NAMES); do echo "  - $$pkg"; done
	@echo "  - macos (make macos.edit を除く)"
	@echo ""
	@echo "[オプション]:"
	@echo "  - NOCACHE=1"
	@echo "  - NOMOUNT=1"
	@echo "  - NOTTY=1"
	@echo ""
	@echo "make <パッケージ名> は、指定したパッケージ名の環境用の momo バイナリを生成します。"
	@echo ""
	@echo "make <パッケージ名>.package は、指定したパッケージ名の環境用の momo バイナリを含んだリリース用パッケージを生成します。"
	@echo ""
	@echo "make <パッケージ名>.edit は、WebRTC のソースを変更して、カスタマイズされた WebRTC を使って momo バイナリを生成するための機能です。"
	@echo "詳細は doc/PATCH_WEBRTC.md を参照して下さい。"
	@echo ""
	@echo "make <パッケージ名>.clean は、<パッケージ名> の Docker イメージやビルドキャッシュを削除します。"
	@echo ""
	@echo "[オプション] は、make NOCACHE=1 <パッケージ名> のように指定します。"
	@echo ""
	@echo "NOCAHCE=1 オプションは、イメージのキャッシュを利用しないモードで動作させることができます。"
	@echo "具体的には docker build コマンドに --no-cache を指定します。--no-cache コマンドの詳細は Docker のドキュメントを参照して下さい。"
	@echo ""
	@echo "NOMOUNT=1 オプションは、マウントを利用しないモードで動作させることができます。"
	@echo "何らかの理由でマウントがうまく動作しない場合に使って下さい。"
	@echo ""
	@echo "NOTTY=1 オプションは、tty を利用しないモードで動作させることができます。"
	@echo "具体的には docker build コマンドに --progress=plain を指定します。"
	@echo ""

WEBRTC_COMMIT_SHORT=$(shell echo $(WEBRTC_COMMIT) | cut -b 1-8)

ifeq ($(NOCACHE),1)
  DOCKER_BUILD_FLAGS += --no-cache
endif

ifeq ($(NOMOUNT),1)
  DOCKER_MOUNT = nomount
else
  DOCKER_MOUNT = mount
endif

ifeq ($(NOTTY),1)
  DOCKER_BUILD_FLAGS += --progress=plain
endif

# パッケージ名からルールを自動生成する
define generateDockerRules

.PHONY: $(1)
$(1): $(1).prepare
	ls -lha ../script
	../script/docker_run.sh `pwd` `pwd`/.. $(DOCKER_MOUNT) $(1) momo/$(1):m$$(WEBRTC_VERSION)-$$(WEBRTC_COMMIT_SHORT) build ${MOMO_VERSION} $(CLI11_VERSION) $(JSON_VERSION)

.PHONY: $(1).package
$(1).package: $(1).prepare
	# momo を package モードでビルドし直す
	../script/docker_run.sh `pwd` `pwd`/.. $(DOCKER_MOUNT) $(1) momo/$(1):m$$(WEBRTC_VERSION)-$$(WEBRTC_COMMIT_SHORT) package ${MOMO_VERSION} $(CLI11_VERSION) $(JSON_VERSION)

	rm -rf package/momo-$(MOMO_VERSION)_$(1)
	rm -f package/momo-$(MOMO_VERSION)_$(1).tar.gz
	mkdir -p package/momo-$(MOMO_VERSION)_$(1)
	cp    ../_build/$(1)/momo package/momo-$(MOMO_VERSION)_$(1)/
	cp    ../LICENSE          package/momo-$(MOMO_VERSION)_$(1)/
	cp    ../NOTICE           package/momo-$(MOMO_VERSION)_$(1)/
	cp -r ../html             package/momo-$(MOMO_VERSION)_$(1)/html
	cd package && tar czf momo-$(MOMO_VERSION)_$(1).tar.gz momo-$(MOMO_VERSION)_$(1)
	rm -rf package/momo-$(MOMO_VERSION)_$(1)
	@echo ""
	@echo "パッケージが package/momo-$(MOMO_VERSION)_$(1).tar.gz に生成されました。"

.PHONY: $(1).edit
$(1).edit: $(1).prepare
	docker run -it --rm -v `pwd`/..:/root/momo momo/$(1):m$$(WEBRTC_VERSION)-$$(WEBRTC_COMMIT_SHORT) /bin/bash -c 'export PACKAGE_NAME=$(1); cd /root/momo && exec /bin/bash'

.PHONY: $(1).prepare
$(1).prepare:
	ls -lha ../script
	rm -rf $(1)/patch
	ls -lha ../script
	rm -rf $(1)/script
	ls -lha ../script
	cp -r ../patch $(1)/patch
	ls -lha ../script
	cp -r ../script $(1)/script
	ls -lha ../script

	# DOCKER_BUILDKIT=1 docker build -t momo/$(1):m$$(WEBRTC_VERSION)-$$(WEBRTC_COMMIT_SHORT) $(DOCKER_BUILD_FLAGS) --build-arg WEBRTC_VERSION=$$(WEBRTC_VERSION) --build-arg WEBRTC_COMMIT=$$(WEBRTC_COMMIT) --build-arg BOOST_VERSION=$$(BOOST_VERSION) --build-arg PACKAGE_NAME=$(1) $(1)

	ls -lha ../script
	rm -r $(1)/patch
	ls -lha ../script
	rm -r $(1)/script

.PHONY: $(1).clean
$(1).clean:
	-docker run -it --rm -v `pwd`/..:/root/momo momo/$(1):m$$(WEBRTC_VERSION)-$$(WEBRTC_COMMIT_SHORT) /bin/bash -c 'cd /root/momo && make PACKAGE_NAME=$(1) clean'
	IMAGES=`docker image ls -q momo/$(1)` && if [ -n "$$$${IMAGES}" ]; then docker image rm $$$${IMAGES}; fi
	docker builder prune -f --filter=label=jp.shiguredo.momo=$(1)

endef

# Docker 用のルールを生成
$(foreach package_name, $(PACKAGE_NAMES), $(eval $(call generateDockerRules,$(package_name))))

.PHONY: macos
macos: macos.prepare
	make -C .. MOMO_CFLAGS="-O2" PACKAGE_NAME=macos MOMO_VERSION=${MOMO_VERSION} momo

.PHONY: macos.package
macos.package: macos.prepare
	# momo を package モードでビルドし直す
	rm -f ../_build/macos/momo && make -C .. MOMO_CFLAGS="-O2" PACKAGE_NAME=macos BUILD_MODE=package momo

	# パッケージのバイナリを作る
	rm -rf package/momo-$(MOMO_VERSION)_macos
	rm -f package/momo-$(MOMO_VERSION)_macos.tar.gz
	mkdir -p package/momo-$(MOMO_VERSION)_macos
	cp    ../_build/macos/momo package/momo-$(MOMO_VERSION)_macos/
	cp    ../LICENSE           package/momo-$(MOMO_VERSION)_macos/
	cp    ../NOTICE            package/momo-$(MOMO_VERSION)_macos/
	cp -r ../html              package/momo-$(MOMO_VERSION)_macos/html
	cd package && tar czf momo-$(MOMO_VERSION)_macos.tar.gz momo-$(MOMO_VERSION)_macos

	rm -rf package/momo-$(MOMO_VERSION)_macos
	@echo ""
	@echo "パッケージが package/momo-$(MOMO_VERSION)_macos.tar.gz に生成されました。"

.PHONY: macos.prepare
macos.prepare:
	make -C macos

.PHONY: macos.clean
macos.clean:
	make -C macos clean
	make -C .. PACKAGE_NAME=macos clean
