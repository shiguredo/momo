#!/usr/bin/env bash
set -euo pipefail

# sysroot を構築し .cargo/config.toml を自動生成する
cargo shiguredo-sysroot --config sysroot/raspberry-pi-os-trixie_arm64.json

# デフォルトターゲットと feature を設定する（cargo build だけで動くようにする）
cat >> .cargo/config.toml <<'EOF'

[build]
target = "aarch64-unknown-linux-gnu"
EOF

echo "セットアップ完了"
echo "以下のコマンドがそのまま動きます:"
echo "  cargo build --features raspberrypi"
echo "  cargo check --features raspberrypi"
echo "  cargo clippy --features raspberrypi"
