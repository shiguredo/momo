# Intel VPL YUY2 サポート

## 概要

Momo は **Intel VPL 限定** で H.264 (AVC) および H.265 (HEVC) エンコードにおいて YUY2 フォーマットの直接入力をサポートしています。これにより、YUY2 出力対応のカメラから取得した映像データを変換なしで直接エンコードでき、高フレームレート（最大 120fps）の実現が可能になります。

> [!IMPORTANT]
> YUY2 サポートは Intel VPL エンコーダーのみです。H.264 と H.265 の両方で利用可能です。

## 動作要件

- **Intel VPL 対応のハードウェア（必須）**
  - Intel® Discrete Graphics
    - Intel® Iris® Xe MAX Graphics
    - Intel® Arc™ A-Series Graphics
- Ubuntu 24.04 x86_64
- YUY2 出力対応のカメラデバイス
- H.264 または H.265 エンコーダーとして Intel VPL を使用
  - `--h264-encoder vpl` または `--h265-encoder vpl` を指定する必要あり

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

### Intel VPL での実装

Intel VPL では以下のプロファイルを使用して YUV422 入力をサポートしています：

- **H.264**: High 4:2:2 プロファイル (MFX_PROFILE_AVC_HIGH_422)
- **H.265**: Range Extension プロファイル (MFX_PROFILE_HEVC_REXT)

これにより、カメラから取得した YUY2 データを **フォーマット変換なし** でエンコードできます。

#### VPL サーフェスバッキング実装

Momo では以下の最適化を実装しています：

1. **VplBackedNativeBuffer**：VPL サーフェスメモリを直接使用する特殊な NativeBuffer
2. **VplSurfacePool**：エンコーダーとキャプチャラー間でサーフェスメモリを共有管理
3. **メモリコピー削減**：V4L2 から VPL サーフェスへの直接コピー（1回）のみ

> [!NOTE]
> VPL サーフェスバッキング機能により、メモリコピーを 2 回から 1 回に削減しています。V4L2 から VPL サーフェスへの直接コピーのみで、エンコーダーへの追加コピーは不要です。

## 使用方法

### コマンドライン オプション

#### --force-yuy2

YUY2 フォーマットの使用を強制します。カメラが YUY2 をサポートしていない場合はエラーで終了します。

```bash
# H.265 エンコードの場合
momo --force-yuy2 \
      --video-device "Elgato Facecam MK.2" \
      --resolution 1280x720 \
      --framerate 120 \
      --h265-encoder vpl \
      --video-codec-type H265 \
      sora ...

# H.264 エンコードの場合
momo --force-yuy2 \
      --video-device "Elgato Facecam MK.2" \
      --resolution 1280x720 \
      --framerate 120 \
      --h264-encoder vpl \
      --video-codec-type H264 \
      sora ...
```

#### --framerate

1 から 120 までのフレームレートを指定できます（デフォルト: 30）。

### 使用例

**高フレームレート配信（120fps）：**

```bash
# H.265 エンコードの場合
momo --force-yuy2 \
      --video-device "Elgato Facecam MK.2" \
      --resolution 1280x720 \
      --framerate 120 \
      --h265-encoder vpl \
      --video-codec-type H265 \
      --video-bit-rate 5000 \
      sora --signaling-urls wss://example.com/signaling \
           --role sendonly \
           --channel-id test-channel

# H.264 エンコードの場合（より広い互換性）
momo --force-yuy2 \
      --video-device "Elgato Facecam MK.2" \
      --resolution 1280x720 \
      --framerate 120 \
      --h264-encoder vpl \
      --video-codec-type H264 \
      --video-bit-rate 5000 \
      sora --signaling-urls wss://example.com/signaling \
           --role sendonly \
           --channel-id test-channel
```

**対応解像度とフレームレートの例（Elgato Facecam MK.2）：**

| 解像度 | 最大フレームレート（YUY2） |
|--------|---------------------------|
| 1280x720 | 120fps |
| 960x540 | 120fps |

## パフォーマンス効果

### 処理フローの最適化

**フォーマット変換が必要な処理フロー：**

```text
カメラ (YUY2) → I420 変換 → NV12 変換 → H.265 エンコード
```

**Momo の処理フロー（VPL サーフェスバッキング）：**

```text
カメラ (YUY2) → VPL サーフェス → H.265 エンコード（メモリコピー1回のみ）
```

### 効果

- **CPU 使用率の大幅削減**：フォーマット変換処理（YUY2→I420→NV12）が不要
- **メモリ帯域の 50% 削減**：メモリコピーを 2 回から 1 回に削減（1280x720@120fps で 442→221 MB/秒）
- **レイテンシの短縮**：変換処理とメモリコピーの削減による処理時間短縮
- **高フレームレート（120fps）の実現**：CPU リソースの節約により高フレームレート処理が可能

## 制限事項

- Intel VPL エンコーダーのみ対応
- H.264 および H.265 コーデック対応
- Ubuntu 24.04 x86_64 のみ対応
- カメラデバイスが YUY2 出力に対応している必要あり
- `--force-yuy2` 使用時は YUY2 が利用できない場合エラーで終了する

## 動作確認

YUY2 サポートが有効になっているかは、ログで確認できます：

**成功時のログ：**

```text
[INFO] Using YUY2 format for H.265/HEVC encoding  # H.265 の場合
[INFO] Using YUY2 format for H.264/AVC encoding    # H.264 の場合
[INFO] Using YUY2 format for video capture (--force-yuy2): 1280x720 @ 120fps
[INFO] VplSurfacePool initialized: 1280x720 format=YUY2 surfaces=10
[VERBOSE] Using VPL-backed surface for YUY2 capture (memory copy reduction)
[VERBOSE] Using VPL-backed NativeBuffer (zero-copy from V4L2 to encoder)
[INFO] V4L2: Capture format set to YUY2 1280x720 @ 120fps
```

**VPL サーフェスプールが利用できない場合：**

```text
[VERBOSE] VPL surface not available, using regular NativeBuffer
[INFO] Using YUY2 data directly from NativeBuffer (no conversion)
```

**エラー時のログ（--force-yuy2 使用時）：**

```text
[ERROR] YUY2 format with 1920x1080 @ 120fps not supported by device
```

**通常動作時（--force-yuy2 未使用）：**

```text
[INFO] Using default I420 format for video capture
```

## トラブルシューティング

### YUY2 フォーマットが使用できない

1. カメラデバイスの YUY2 サポートを確認
2. Intel VPL が正しくインストールされているか確認
3. --h264-encoder vpl または --h265-encoder vpl が指定されているか確認

### 期待したフレームレートが出ない

1. カメラの仕様で指定解像度での最大フレームレートを確認
2. --framerate オプションで適切な値を指定
3. ログで実際のキャプチャーフレームレートを確認

## 動作確認済みカメラ

- [Elgato Facecam MK.2](https://www.elgato.com/us/en/p/facecam-mk2)
  - 1280x720 @ 120fps (YUY2)
  - 960x540 @ 120fps (YUY2)
  - [技術仕様](https://help.elgato.com/hc/en-us/articles/24162700661517-Elgato-Facecam-MK-2-Technical-Specifications)

## 参考資料

- [Intel® oneVPL Hardware Support Details](https://www.intel.com/content/www/us/en/docs/onevpl/developer-reference-media-intel-hardware/1-1/details.html)
