.PHONY: test cover pbt pbt-cover fuzz fuzzing fuzzing-parallel fuzzing-list check clippy fmt clean \
	sysroot-raspberry-pi sysroot-ubuntu-24.04_arm64 sysroot-ubuntu-22.04_arm64 \
	sysroot-clippy-raspberry-pi sysroot-clippy-ubuntu-24.04_arm64 sysroot-clippy-ubuntu-22.04_arm64 \
	sysroot-build-raspberry-pi sysroot-build-raspberry-pi-release \
	sysroot-build-ubuntu-24.04_arm64 sysroot-build-ubuntu-24.04_arm64-release \
	sysroot-build-ubuntu-22.04_arm64 sysroot-build-ubuntu-22.04_arm64-release \
	clippy-raspberry-pi

# 全テストを実行する
test:
	cargo test --workspace

# 全テストカバレッジ付きで実行する
cover:
	cargo llvm-cov --tests --workspace

# PBT をカバレッジ付きで実行する
pbt-with-cover:
	cargo llvm-cov -p pbt --tests

# Fuzzing を全ターゲットで逐次実行する（fork 数はコア数に応じて自動調整）
fuzzing:
	@FORKS=$$(( $$(nproc) - 2 )); \
	if [ $$FORKS -lt 1 ]; then FORKS=1; fi; \
	echo "Using fork=$$FORKS on $$(nproc) cores"; \
	for target in $$(cargo fuzz list); do \
		echo "=== Fuzzing $$target ==="; \
		cargo +nightly fuzz run $$target -- -max_total_time=30 -fork=$$FORKS -max_len=4096 || exit 1; \
	done

# Fuzzing を全ターゲットで並列実行しレポートを出力する（fork 数はコア数に応じて自動調整）
fuzzing-parallel:
	@mkdir -p fuzz/logs
	@FORKS=$$(( $$(nproc) - 2 )); \
	if [ $$FORKS -lt 1 ]; then FORKS=1; fi; \
	echo "Using fork=$$FORKS on $$(nproc) cores"; \
	cargo fuzz list | xargs -P $$(cargo fuzz list | wc -l) -I {} \
		sh -c 'cargo +nightly fuzz run {} -- -max_total_time=30 -fork=1 -max_len=4096 > fuzz/logs/{}.log 2>&1'
	@echo "=== Fuzzing Report ==="
	@for f in fuzz/logs/*.log; do \
		target=$$(basename $$f .log); \
		last=$$(grep -E '^#[0-9]+:' $$f | tail -1); \
		echo "$$target: $$last"; \
	done

# Fuzzing ターゲット一覧を表示する
fuzzing-list:
	cargo fuzz list

# cargo check を実行する
check:
	cargo check --workspace

# cargo clippy を実行する
clippy:
	cargo clippy --workspace -- -D warnings

# cargo fmt を実行する
fmt:
	cargo fmt --all

# ビルド成果物を削除する
clean:
	cargo clean

# --- sysroot 用変数 ---

# sysroot 環境変数を生成する関数
# .cargo/config.toml に依存せず、全設定を環境変数で渡す
# $(1): sysroot の絶対パス
# $(2): アーキテクチャの lib パス (aarch64-linux-gnu)
# $(3): リンカ名 (aarch64-linux-gnu-gcc)
# $(4): Cargo ターゲット環境変数プレフィックス (AARCH64_UNKNOWN_LINUX_GNU)
# $(5): sysroot bin ディレクトリの絶対パス
# $(6): cc-rs 用ターゲットトリプル (aarch64_unknown_linux_gnu)
sysroot_env = \
	SYSROOT_PATH=$(1) \
	PKG_CONFIG_SYSROOT_DIR=$(1) \
	PKG_CONFIG_PATH=$(1)/usr/lib/$(2)/pkgconfig:$(1)/usr/share/pkgconfig \
	CARGO_TARGET_$(4)_LINKER=$(3) \
	CARGO_TARGET_$(4)_RUSTFLAGS="-C link-arg=--sysroot=$(1)" \
	CC_$(6)=$(5)/$(3)-with-sysroot.sh \
	CXX_$(6)=$(5)/$(subst gcc,g++,$(3))-with-sysroot.sh

# Raspberry Pi
RASPBERRYPI_SYSROOT = $(CURDIR)/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/sysroot
RASPBERRYPI_BIN = $(CURDIR)/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/bin
RASPBERRYPI_ENV = $(call sysroot_env,$(RASPBERRYPI_SYSROOT),aarch64-linux-gnu,aarch64-linux-gnu-gcc,AARCH64_UNKNOWN_LINUX_GNU,$(RASPBERRYPI_BIN),aarch64_unknown_linux_gnu) \
	WEBRTC_C_TARGET=raspberry-pi-os_armv8 \
	WEBRTC_C_SYSROOT=$(RASPBERRYPI_SYSROOT)

# Ubuntu 24.04 arm64
UBUNTU_2404_ARM64_SYSROOT = $(CURDIR)/target/shiguredo-sysroot/ubuntu-24.04_arm64/sysroot
UBUNTU_2404_ARM64_BIN = $(CURDIR)/target/shiguredo-sysroot/ubuntu-24.04_arm64/bin
UBUNTU_2404_ARM64_ENV = $(call sysroot_env,$(UBUNTU_2404_ARM64_SYSROOT),aarch64-linux-gnu,aarch64-linux-gnu-gcc,AARCH64_UNKNOWN_LINUX_GNU,$(UBUNTU_2404_ARM64_BIN),aarch64_unknown_linux_gnu) \
	WEBRTC_C_TARGET=ubuntu-24.04_armv8 \
	WEBRTC_C_SYSROOT=$(UBUNTU_2404_ARM64_SYSROOT)

# Ubuntu 22.04 arm64
UBUNTU_2204_ARM64_SYSROOT = $(CURDIR)/target/shiguredo-sysroot/ubuntu-22.04_arm64/sysroot
UBUNTU_2204_ARM64_BIN = $(CURDIR)/target/shiguredo-sysroot/ubuntu-22.04_arm64/bin
UBUNTU_2204_ARM64_ENV = $(call sysroot_env,$(UBUNTU_2204_ARM64_SYSROOT),aarch64-linux-gnu,aarch64-linux-gnu-gcc,AARCH64_UNKNOWN_LINUX_GNU,$(UBUNTU_2204_ARM64_BIN),aarch64_unknown_linux_gnu) \
	WEBRTC_C_TARGET=ubuntu-22.04_armv8 \
	WEBRTC_C_SYSROOT=$(UBUNTU_2204_ARM64_SYSROOT)

# --- sysroot 生成 ---

# Raspberry Pi 用 sysroot を生成する
sysroot-raspberry-pi:
	cargo shiguredo-sysroot --config sysroot/raspberry-pi-os-trixie_arm64.json
	rm -f .cargo/config.toml

# Ubuntu 24.04 arm64 用 sysroot を生成する
sysroot-ubuntu-24.04_arm64:
	cargo shiguredo-sysroot --config sysroot/ubuntu-24.04_arm64.json
	rm -f .cargo/config.toml

# Ubuntu 22.04 arm64 用 sysroot を生成する
sysroot-ubuntu-22.04_arm64:
	cargo shiguredo-sysroot --config sysroot/ubuntu-22.04_arm64.json
	rm -f .cargo/config.toml

# --- sysroot clippy ---

# Raspberry Pi 向け sysroot clippy する
sysroot-clippy-raspberry-pi:
	$(RASPBERRYPI_ENV) cargo clippy --target aarch64-unknown-linux-gnu --no-default-features --features ayame,sora,raspberrypi -- -D warnings

# Ubuntu 24.04 arm64 向け sysroot clippy する
sysroot-clippy-ubuntu-24.04_arm64:
	$(UBUNTU_2404_ARM64_ENV) cargo clippy --target aarch64-unknown-linux-gnu --no-default-features --features ayame,sora -- -D warnings

# Ubuntu 22.04 arm64 向け sysroot clippy する
sysroot-clippy-ubuntu-22.04_arm64:
	$(UBUNTU_2204_ARM64_ENV) cargo clippy --target aarch64-unknown-linux-gnu --no-default-features --features ayame,sora -- -D warnings

# --- container を使った clippy ---

# container 内で Raspberry Pi 向け clippy を実行する
# sysroot が未構築の場合はコンテナ内で自動構築する
# コンテナ内ではプロジェクトルートが /workspace にマウントされる
clippy-raspberry-pi:
	container build -t momo-dev -f .devcontainer/Dockerfile .devcontainer
	container run --rm --cpus 8 --memory 16g \
		-v $(CURDIR):/workspace -w /workspace \
		-e PKG_CONFIG_ALLOW_CROSS=1 \
		-e PKG_CONFIG_SYSROOT_DIR=/workspace/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/sysroot \
		-e PKG_CONFIG_PATH=/workspace/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/sysroot/usr/lib/aarch64-linux-gnu/pkgconfig:/workspace/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/sysroot/usr/share/pkgconfig \
		-e CC_aarch64_unknown_linux_gnu=/workspace/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/bin/aarch64-linux-gnu-gcc-with-sysroot.sh \
		-e CXX_aarch64_unknown_linux_gnu=/workspace/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/bin/aarch64-linux-gnu-g++-with-sysroot.sh \
		momo-dev bash -c ' \
			export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc ; \
			export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_RUSTFLAGS="-C link-arg=--sysroot=/workspace/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/sysroot" ; \
			export BINDGEN_EXTRA_CLANG_ARGS_aarch64_unknown_linux_gnu="--target=aarch64-linux-gnu --sysroot=/workspace/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/sysroot" ; \
			export WEBRTC_C_TARGET=raspberry-pi-os_armv8 ; \
			export WEBRTC_C_SYSROOT=/workspace/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/sysroot ; \
			if [ ! -d "target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64/sysroot/usr/include" ]; then \
				echo "=== sysroot を構築しています ===" ; \
				CARGO_TARGET_DIR=/tmp/target cargo shiguredo-sysroot --config sysroot/raspberry-pi-os-trixie_arm64.json ; \
				mkdir -p target/shiguredo-sysroot ; \
				cp -r /tmp/target/shiguredo-sysroot/raspberry-pi-os-trixie_arm64 target/shiguredo-sysroot/ ; \
				rm -f .cargo/config.toml ; \
				echo "=== sysroot 構築完了 ===" ; \
			fi ; \
			cargo clippy --target aarch64-unknown-linux-gnu --no-default-features --features ayame,sora,raspberrypi -- -D warnings \
		'

# --- sysroot ビルド ---

# Raspberry Pi 向け sysroot ビルドする
sysroot-build-raspberry-pi:
	$(RASPBERRYPI_ENV) cargo build --target aarch64-unknown-linux-gnu --no-default-features --features ayame,sora,raspberrypi

# Raspberry Pi 向け sysroot リリースビルドする
sysroot-build-raspberry-pi-release:
	$(RASPBERRYPI_ENV) cargo build --target aarch64-unknown-linux-gnu --no-default-features --features ayame,sora,raspberrypi --release

# Ubuntu 24.04 arm64 向け sysroot ビルドする
sysroot-build-ubuntu-24.04_arm64:
	$(UBUNTU_2404_ARM64_ENV) cargo build --target aarch64-unknown-linux-gnu --no-default-features --features ayame,sora

# Ubuntu 24.04 arm64 向け sysroot リリースビルドする
sysroot-build-ubuntu-24.04_arm64-release:
	$(UBUNTU_2404_ARM64_ENV) cargo build --target aarch64-unknown-linux-gnu --no-default-features --features ayame,sora --release

# Ubuntu 22.04 arm64 向け sysroot ビルドする
sysroot-build-ubuntu-22.04_arm64:
	$(UBUNTU_2204_ARM64_ENV) cargo build --target aarch64-unknown-linux-gnu --no-default-features --features ayame,sora

# Ubuntu 22.04 arm64 向け sysroot リリースビルドする
sysroot-build-ubuntu-22.04_arm64-release:
	$(UBUNTU_2204_ARM64_ENV) cargo build --target aarch64-unknown-linux-gnu --no-default-features --features ayame,sora --release
