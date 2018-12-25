.PHONY: all
all: clean
	cp -r ../../patch patch

	# x86_64 用 momo を生成するために必要なデータだけ入ったイメージ
	docker inspect momo/webrtc-x86_64_ros:m$(WEBRTC_VERSION) > /dev/null 2>&1 || docker image build --rm -t momo/webrtc-x86_64_ros:m$(WEBRTC_VERSION) --build-arg WEBRTC_VERSION=$(WEBRTC_VERSION) .

	rm -r patch/

	git clone ../.. momo
	tar czf momo.tar.gz momo
	rm -rf momo

	# ベースイメージから構築したコンテナに転送してビルドし、
	# ビルドが完了したら成果物の momo バイナリだけ取り出す
	docker container create -it --name momo momo/webrtc-x86_64_ros:m$(WEBRTC_VERSION)
	docker container start momo
	docker container cp momo.tar.gz momo:/root/
	docker container exec momo /bin/bash -c 'cd /root && tar xf momo.tar.gz && cd momo && make OUT_PATH=x86_64_ros ROS_VERSION=kinetic BUILD_MODE=$(BUILD_MODE) MOMO_VERSION=$(MOMO_VERSION)'
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
clean.image:
	-docker image rm momo/webrtc-x86_64_ros:m$(WEBRTC_VERSION)

.PHONY: clean.image.all
clean.image.all:
	-docker image rm momo/webrtc-x86_64_ros:m$(WEBRTC_VERSION)
