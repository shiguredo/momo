# Intel VPL DMABUF ゼロコピー実装

ブランチ: `feature/intel-vpl-dmabuf-zero-copy`

## 概要

V4L2 から Intel VPL へ DMABUF を使用した完全ゼロコピーパイプラインの実装。
すべてのビデオ処理が GPU メモリ上で完結し、CPU コピーを排除。

## アーキテクチャ

```text
V4L2 Camera → DMABUF → VPP (GPU) → Encoder (GPU) → Output
   (YUY2)               (YUY2→NV12)  (NV12→AV1)
```

## 処理フロー

1. **V4L2 キャプチャ**: V4L2_MEMORY_DMABUF で VPL の DMABUF に直接書き込み
2. **VPP 色空間変換**: GPU 上で YUY2 → NV12 変換
3. **エンコード**: GPU 上で NV12 → AV1/H.265 エンコード

## 使用方法

### Sora モード（AV1）

```bash
./momo --use-vpl-dmabuf \
       --no-audio-device \
       --video-device "Elgato Facecam MK.2" \
       --resolution 1280x720 \
       --framerate 30 \
       --force-yuy2 \
       --av1-encoder vpl \
       sora --signaling-urls wss://sora.example.com/signaling \
       --role sendonly \
       --channel-id CHANNEL_ID \
       --video-codec-type AV1 \
       --video-bit-rate 2500
```

### Sora モード（H.265）

```bash
./momo --use-vpl-dmabuf \
       --no-audio-device \
       --video-device "Elgato Facecam MK.2" \
       --resolution 1280x720 \
       --framerate 30 \
       --force-yuy2 \
       --h265-encoder vpl \
       sora --signaling-urls wss://sora.example.com/signaling \
       --role sendonly \
       --channel-id CHANNEL_ID \
       --video-codec-type H265 \
       --video-bit-rate 2500
```

### Test モード（AV1）

```bash
./momo --use-vpl-dmabuf \
       --no-audio-device \
       --video-device "Elgato Facecam MK.2" \
       --resolution 1280x720 \
       --framerate 30 \
       --force-yuy2 \
       --av1-encoder vpl \
       test
```

### Test モード（H.265）

```bash
./momo --use-vpl-dmabuf \
       --no-audio-device \
       --video-device "Elgato Facecam MK.2" \
       --resolution 1280x720 \
       --framerate 30 \
       --force-yuy2 \
       --h265-encoder vpl \
       test
```

### 重要なオプション

- `--use-vpl-dmabuf`: DMABUF ゼロコピーモードを有効化
- `--force-yuy2`: YUY2 フォーマットを強制（VPP で変換するため）
- `--av1-encoder vpl`: AV1 用 VPL エンコーダーを使用
- `--h265-encoder vpl`: H.265 用 VPL エンコーダーを使用
- `--video-codec-type AV1`: AV1 または H.265 コーデック

## 要件

- Ubuntu 24.04 x86_64 専用
- Intel GPU (VA-API サポート)
- Intel VPL 2.x 以降
- V4L2 対応カメラ (YUY2 出力サポート)

## パフォーマンス

- **CPU 使用率**: 色空間変換を GPU で実行することで大幅削減
- **レイテンシ**: メモリコピー排除により低減
- **メモリ帯域**: システムメモリ転送なし

## 制限事項

- Ubuntu 24.04 x86_64 専用
- Intel GPU 必須
- カメラは YUY2 出力対応が必要
