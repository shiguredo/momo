FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    curl \
    libclang-dev \
    clang \
    pkg-config \
    make \
    debian-archive-keyring \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    git \
    && rm -rf /var/lib/apt/lists/*

# Debian の keyring を配置する（trixie リポジトリの署名検証用）
RUN install -Dm644 /usr/share/keyrings/debian-archive-keyring.gpg \
    /etc/apt/trusted.gpg.d/debian-archive-keyring.gpg

# Raspberry Pi アーカイブの keyring をインストールし trusted.gpg.d に配置する
RUN curl -fsSL http://archive.raspberrypi.com/debian/pool/main/r/raspberrypi-archive-keyring/raspberrypi-archive-keyring_2021.1.1+rpt1_all.deb \
    -o /tmp/rpi-keyring.deb && dpkg -i /tmp/rpi-keyring.deb && rm /tmp/rpi-keyring.deb \
    && install -Dm644 /usr/share/keyrings/raspberrypi-archive-keyring.gpg \
    /etc/apt/trusted.gpg.d/raspberrypi-archive-keyring.gpg

# Rust をインストールする
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"

RUN rustup target add aarch64-unknown-linux-gnu

# cargo-shiguredo-sysroot をインストールする
RUN cargo install shiguredo_sysroot
