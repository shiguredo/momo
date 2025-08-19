# Intel VPL H.265 YUY2 サポート

## 概要

Momo は Intel VPL を使用した H.265 (HEVC) エンコードにおいて、YUY2 フォーマットの直接入力をサポートしています。これにより、YUY2 出力対応のカメラから取得した映像データを変換なしで直接エンコードでき、高フレームレート（1080p 120fps など）の実現が可能になります。

## 動作要件

- Intel VPL 対応のハードウェア（Intel Xe2/Xe-HPM 世代以降推奨）
- YUY2 出力対応のカメラデバイス
- H.265 エンコーダーの使用

## 技術詳細

### YUY2 フォーマット詳細

#### YUV サンプリング方式について

YUV カラーフォーマットは、輝度（Y）と色差（U, V）の情報を分離して保存します。サンプリング方式は `J:a:b` 形式で表現されます：

- **YUV420 (4:2:0)**：水平・垂直方向ともに色差信号を 2:1 でサンプリング
- **YUV422 (4:2:2)**：水平方向のみ色差信号を 2:1 でサンプリング、垂直方向はフル解像度
- **YUV444 (4:4:4)**：全ての成分をフル解像度でサンプリング

#### YUY2 の具体的な構造

YUY2 は YUV422 サンプリング方式のパックドフォーマット実装です：

**メモリレイアウト：**

```text
バイト位置: [0]  [1]  [2]  [3]  [4]  [5]  [6]  [7]  ...
内容:      Y0   U0   Y1   V0   Y2   U2   Y3   V2   ...
ピクセル:  [ピクセル0] [ピクセル1] [ピクセル2] [ピクセル3]
```

**特徴：**

- 2 ピクセルを 4 バイト（32 ビット）で表現
- 輝度（Y）は各ピクセルごとに独立した値
- 色差（U, V）は水平方向の 2 ピクセルで共有
- 1 ピクセルあたり平均 16 ビット（2 バイト）

**他のフォーマットとの比較：**

| フォーマット | サンプリング | ビット/ピクセル | メモリレイアウト | 用途 |
|------------|------------|---------------|----------------|------|
| I420 (YUV420) | 4:2:0 | 12 | プレーナー | 一般的な動画圧縮 |
| NV12 (YUV420) | 4:2:0 | 12 | セミプレーナー | ハードウェアエンコード |
| YUY2 (YUV422) | 4:2:2 | 16 | パックド | 高品質キャプチャー |
| YUV444 | 4:4:4 | 24 | プレーナー | ロスレス処理 |

#### Intel VPL での実装

Intel VPL では、YUY2 は `MFX_FOURCC_YUY2` として定義され、`MFX_CHROMAFORMAT_YUV422` クロマフォーマットと関連付けられています。

### 実装方式

#### 1. HEVC REXT プロファイル

HEVC (H.265) の Range Extension プロファイルを使用して YUV422 入力をサポート：

```cpp
// vpl_video_encoder.cpp での実装
if (codec == MFX_CODEC_HEVC) {
    param.mfx.FrameInfo.FourCC = MFX_FOURCC_YUY2;
    param.mfx.FrameInfo.ChromaFormat = MFX_CHROMAFORMAT_YUV422;
    param.mfx.CodecProfile = MFX_PROFILE_HEVC_REXT;
}
```

**サポートされるプロファイル（Intel VPL）：**

- `MFX_PROFILE_HEVC_MAIN`：NV12 (YUV420) のみ
- `MFX_PROFILE_HEVC_REXT`：YUY2 (YUV422)、AYUV (YUV444) などをサポート

#### 2. ゼロコピー転送の実現

**NativeBuffer を使用した直接転送：**

```cpp
// v4l2_video_capturer.cpp での実装
if (_captureVideoType == webrtc::VideoType::kYUY2) {
    auto native_buffer = NativeBuffer::Create(
        webrtc::VideoType::kYUY2, _currentWidth, _currentHeight);
    memcpy(native_buffer->MutableData(), data, bytesused);
    native_buffer->SetLength(bytesused);
    dst_buffer = native_buffer;
}
```

**エンコーダー側での処理：**

```cpp
// vpl_video_encoder.cpp での実装
if (use_yuy2_ && video_frame_buffer->type() == webrtc::VideoFrameBuffer::Type::kNative) {
    auto native_buffer = reinterpret_cast<const NativeBuffer*>(video_frame_buffer.get());
    if (native_buffer->VideoType() == webrtc::VideoType::kYUY2) {
        // YUY2 データを直接コピー（変換なし）
        memcpy(surface->Data.Y, native_buffer->Data(), native_buffer->Length());
    }
}
```

#### 3. サーフェスメモリ管理

YUY2 用のサーフェス設定：

```cpp
if (use_yuy2_) {
    // YUY2 は 1 ピクセルあたり 16 ビット (2 バイト)
    size = width * height * 2;
    
    // パックドフォーマットのポインタ設定
    surface.Data.Y = surface_buffer_.data() + i * size;
    surface.Data.U = surface.Data.Y + 1;  // U は Y の次のバイト
    surface.Data.V = surface.Data.Y + 3;  // V は Y から 3 バイト目
    surface.Data.Pitch = width * 2;       // ピッチは幅の 2 倍
}
```

#### 4. 自動フォーマット選択

カメラキャプチャー時の優先順位：

```cpp
// device_video_capturer.cpp での実装
capability_.videoType = webrtc::VideoType::kYUY2;
if (vcm_->StartCapture(capability_) != 0) {
    // YUY2 が失敗した場合は I420 にフォールバック
    capability_.videoType = webrtc::VideoType::kI420;
}
```

## パフォーマンス効果

従来の処理フロー：

```text
カメラ (YUY2) → I420 変換 → NV12 変換 → H.265 エンコード
```

最適化後の処理フロー：

```text
カメラ (YUY2) → H.265 エンコード（直接）
```

この最適化により、以下の効果が期待できます：

- CPU 使用率の削減（変換処理の省略）
- レイテンシの短縮
- 高フレームレート（120fps）の実現

## 制限事項

- H.265 エンコーダー限定（H.264、VP9、AV1 では利用不可）
- Intel VPL ハードウェアエンコーダーが必要
- カメラデバイスが YUY2 出力に対応している必要がある

## 動作確認

YUY2 サポートが有効になっているかは、ログで確認できます：

```text
Using YUY2 format for HEVC encoding
Using YUY2 format for video capture
Using YUY2 data directly from NativeBuffer (no conversion)
```

YUY2 がサポートされていない場合：

```text
YUY2 not supported for HEVC, falling back to NV12
YUY2 capture not supported, falling back to I420
```
