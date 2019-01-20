include ../VERSION

.PHONY: help
help:
	@echo "各環境用の momo バイナリやパッケージを作るための Makefile です。"
	@echo ""
	@echo "使い方:"
	@echo "  make <パッケージ名>"
	@echo "  make <パッケージ名>.package"
	@echo "  make <パッケージ名>.edit"
	@echo "  make clean"
	@echo ""
	@echo "<パッケージ名>:"
	@for pkg in $(PACKAGE_NAMES); do echo "  - $$pkg"; done
	@echo "  - mac (make mac.edit を除く)"
	@echo ""
	@echo "make <パッケージ名> は、指定したパッケージ名の環境用の momo バイナリを生成します。"
	@echo ""
	@echo "make <パッケージ名>.package は、指定したパッケージ名の環境用の momo バイナリを含んだリリース用パッケージを生成します。"
	@echo ""
	@echo "make <パッケージ名>.edit は、WebRTC のソースを変更して、カスタマイズされた WebRTC を使って momo バイナリを生成するための機能です。"
	@echo "make <パッケージ名>.edit を実行すると Docker コンテナの中に入ります。"
	@echo "その中で make を打つと momo バイナリが生成できます（make <パッケージ名> としたのと同じバイナリが生成されます）。"
	@echo "また、/root/webrtc/src 以下にあるソースを書き換えてから /root/momo ディレクトリで make すると、カスタマイズされた WebRTC を使って momo バイナリが生成できます。"
	@echo ""
	@echo "make clean は、*全ての* Docker イメージとキャッシュを削除します。"
	@echo "そのため、momo と無関係なイメージやキャッシュがある場合には注意してください。"
	@echo ""

WEBRTC_COMMIT_SHORT=$(shell echo $(WEBRTC_COMMIT) | cut -b 1-8)

# パッケージ名からルールを自動生成する
define generateDockerRules

.PHONY: $(1)
$(1): $(1).prepare
	docker run -it --rm -v `pwd`/..:/root/momo momo/$(1):m$$(WEBRTC_VERSION)-$$(WEBRTC_COMMIT_SHORT) /bin/bash -c 'cd /root/momo && make MOMO_CFLAGS="-O2" PACKAGE_NAME=$(1) momo'

.PHONY: $(1).package
$(1).package: $(1).prepare
	# momo を package モードでビルドし直す
	docker run -it --rm -v `pwd`/..:/root/momo momo/$(1):m$$(WEBRTC_VERSION)-$$(WEBRTC_COMMIT_SHORT) /bin/bash -c 'cd /root/momo && rm -f _build/$(1)/momo && make MOMO_CFLAGS="-O2" PACKAGE_NAME=$(1) BUILD_MODE=package momo'

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
	rm -rf $(1)/patch
	rm -rf $(1)/script
	cp -r ../patch $(1)/patch
	cp -r ../script $(1)/script

	DOCKER_BUILDKIT=1 docker build -t momo/$(1):m$$(WEBRTC_VERSION)-$$(WEBRTC_COMMIT_SHORT) --build-arg WEBRTC_VERSION=$$(WEBRTC_VERSION) --build-arg WEBRTC_COMMIT=$$(WEBRTC_COMMIT) --build-arg BOOST_VERSION=$$(BOOST_VERSION) --build-arg PACKAGE_NAME=$(1) $(1)

	rm -r $(1)/patch
	rm -r $(1)/script

endef

# 全ての Docker イメージとキャッシュの削除
.PHONY: clean
clean:
	docker container prune -f
	docker image prune -a -f
	docker builder prune -f

# Docker 用のルールを生成
$(foreach package_name, $(PACKAGE_NAMES), $(eval $(call generateDockerRules,$(package_name))))

.PHONY: mac
mac: mac.prepare
	make -C .. MOMO_CFLAGS="-O2" PACKAGE_NAME=mac momo

.PHONY: mac.package
mac.package: mac.prepare
	# momo を package モードでビルドし直す
	rm -f ../_build/mac/momo && make -C .. MOMO_CFLAGS="-O2" PACKAGE_NAME=mac BUILD_MODE=package momo

	# パッケージのバイナリを作る
	rm -rf package/momo-$(MOMO_VERSION)_mac
	rm -f package/momo-$(MOMO_VERSION)_mac.tar.gz
	mkdir -p package/momo-$(MOMO_VERSION)_mac
	cp    ../_build/mac/momo package/momo-$(MOMO_VERSION)_mac/
	cp    ../LICENSE         package/momo-$(MOMO_VERSION)_mac/
	cp    ../NOTICE          package/momo-$(MOMO_VERSION)_mac/
	cp -r ../html            package/momo-$(MOMO_VERSION)_mac/html
	cd package && tar czf momo-$(MOMO_VERSION)_mac.tar.gz momo-$(MOMO_VERSION)_mac

	rm -rf package/momo-$(MOMO_VERSION)_mac
	@echo ""
	@echo "パッケージが package/momo-$(MOMO_VERSION)_mac.tar.gz に生成されました。"

.PHONY: mac.prepare
mac.prepare:
	make -C mac

.PHONY: mac.clean
mac.clean:
	make -C mac clean
