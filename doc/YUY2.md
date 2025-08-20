# YUY2 サポート

## 概要

Momo は YUY2 (YUV422) フォーマットの直接キャプチャーをサポートしています。これにより、YUY2 出力対応のカメラから取得した映像データの変換オーバーヘッドを削減し、高フレームレート（最大 120fps）の実現が可能になります。

## 動作要件

- YUY2 出力対応のカメラデバイス
- Ubuntu / Raspberry Pi OS / macOS

## 技術詳細

### YUY2 フォーマット詳細

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

### 実装詳細

Momo では以下の最適化を実装しています：

1. **NativeBuffer**: WebRTC のビデオパイプライン内で YUY2 データを変換なしで保持
2. **メモリコピー削減**: V4L2 から NativeBuffer への直接コピーのみ
3. **遅延変換**: エンコーダーが YUY2 をサポートしない場合のみ I420 に変換

## 使用方法

### コマンドライン オプション

#### --force-yuy2

YUY2 フォーマットの使用を強制します。カメラが YUY2 をサポートしていない場合はエラーで終了します。

```bash
momo --force-yuy2 \
      --video-device "Elgato Facecam MK.2" \
      --resolution 1280x720 \
      --framerate 120 \
      sora ...
```

#### --framerate

1 から 120 までのフレームレートを指定できます（デフォルト: 30）。

### 使用例

**高フレームレート配信（120fps）：**

```bash
momo --force-yuy2 \
      --video-device "Elgato Facecam MK.2" \
      --resolution 1280x720 \
      --framerate 120 \
      --video-codec-type H264 \
      --video-bit-rate 5000 \
      sora --signaling-urls wss://example.com/signaling \
           --role sendonly \
           --channel-id test-channel
```

## パフォーマンス効果

### 処理フローの最適化

**通常の処理フロー：**

```text
カメラ (YUY2) → I420 変換 → エンコード
```

**Momo の YUY2 最適化フロー：**

```text
カメラ (YUY2) → NativeBuffer → エンコード（変換は必要時のみ）
```

### 効果

- **CPU 使用率の削減**: YUY2→I420 変換処理の削減
- **レイテンシの短縮**: 変換処理の削減による処理時間短縮
- **高フレームレート（120fps）の実現**: CPU リソースの節約により高フレームレート処理が可能

## 制限事項

- カメラデバイスが YUY2 出力に対応している必要あり
- `--force-yuy2` 使用時は YUY2 が利用できない場合エラーで終了する
- ハードウェアエンコーダーでは最終的に I420 または NV12 への変換が必要

## 動作確認

YUY2 サポートが有効になっているかは、ログで確認できます：

**成功時のログ：**

```text
[VERBOSE] Using NativeBuffer for YUY2 data (no conversion)
```

**エラー時のログ（--force-yuy2 使用時）：**

```text
[ERROR] YUY2 format not supported by device
```

## トラブルシューティング

### YUY2 フォーマットが使用できない

1. カメラデバイスの YUY2 サポートを確認
2. 解像度とフレームレートの組み合わせが YUY2 でサポートされているか確認

### 期待したフレームレートが出ない

1. カメラの仕様で指定解像度での最大フレームレートを確認
2. --framerate オプションで適切な値を指定
3. ログで実際のキャプチャーフレームレートを確認

## 動作確認済みカメラ

- [Elgato Facecam MK.2](https://www.elgato.com/us/en/p/facecam-mk2)
  - 1280x720 @ 120fps (YUY2)
  - 960x540 @ 120fps (YUY2)
