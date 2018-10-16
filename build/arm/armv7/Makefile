.PHONY: all
all: clean
	cp -r ../../../patch patch

	# armv7 全体で共通のビルド用イメージ
	docker image build --rm -t momo/webrtc-armv7-buildbase . --file buildbase.Dockerfile

	# armv7 用 momo を生成するために必要なデータだけ入ったイメージ
	docker inspect momo/webrtc-armv7:m$(WEBRTC_VERSION) > /dev/null 2>&1 || docker image build --rm -t momo/webrtc-armv7:m$(WEBRTC_VERSION) --build-arg WEBRTC_VERSION=$(WEBRTC_VERSION) .

	rm -r patch/

	git clone ../../.. momo
	tar czf momo.tar.gz momo
	rm -rf momo

	# ベースイメージから構築したコンテナに転送してビルドし、
	# ビルドが完了したら成果物の momo バイナリだけ取り出す
	docker container create -it --name momo momo/webrtc-armv7:m$(WEBRTC_VERSION)
	docker container start momo
	docker container cp momo.tar.gz momo:/root/
	docker container exec momo /bin/bash -c "cd /root && tar xf momo.tar.gz && cd momo && make USE_IL_ENCODER=$(USE_IL_ENCODER) BUILD_MODE=$(BUILD_MODE)"
	docker container cp momo:/root/momo/momo .
	docker container stop momo
	docker container rm momo

	rm momo.tar.gz
	docker image prune -f
	docker container prune -f

.PHONY: clean
clean:
	# ゴミ掃除する
	-docker container stop momo > /dev/null 2>&1
	-docker container rm momo > /dev/null 2>&1
	-rm -rf momo/ > /dev/null 2>&1
	-rm momo > /dev/null 2>&1
	-rm momo.tar.gz > /dev/null 2>&1
	-rm -r patch/ > /dev/null 2>&1

.PHONY: clean.image
clean.image: clean
	-docker image rm momo/webrtc-armv7:m$(WEBRTC_VERSION)

.PHONY: clean.image.all
clean.image.all: clean
	-docker image rm momo/webrtc-armv7-buildbase
	-docker image rm momo/webrtc-armv7:m$(WEBRTC_VERSION)
