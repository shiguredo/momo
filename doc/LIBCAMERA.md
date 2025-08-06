# libcamera を利用した Momo の設定

## 概要

Momo は Raspberry Pi OS 64bit 環境で libcamera を利用したカメラ入力をサポートしています。
libcamera は Linux 向けの新しいカメラスタックで、従来の V4L2 よりも高度なカメラ制御が可能です。

## 基本的な使い方

### libcamera の有効化

```bash
./momo --use-libcamera sora ...
```

### ネイティブバッファの利用

H.264 エンコード時のみ、メモリコピーを削減する最適化が利用できます：

```bash
./momo --use-libcamera --use-libcamera-native sora ...
```

**制約事項:**

- `--use-libcamera-native` は H.264 かつサイマルキャストが無効の場合のみ有効
- ハードウェアエンコーダーとの組み合わせで最大の効果を発揮

## libcamera コントロールの設定

libcamera は 75 以上のカメラパラメーターを提供しています。Momo では `--libcamera-control` オプションで設定できます。

### 基本的な形式

```bash
--libcamera-control <キー> <値>
```

複数のコントロールを設定する場合：

```bash
./momo --use-libcamera \
       --libcamera-control ExposureTime 10000 \
       --libcamera-control AnalogueGain 2.0 \
       --libcamera-control AfMode Continuous \
       sora ...
```

### 対応している値の形式

#### 基本型

- **bool**: `0` または `1`、`true` または `false` ✅

  ```bash
  --libcamera-control AeEnable 1
  --libcamera-control AeEnable true
  ```

- **int32**: 整数値 ✅

  ```bash
  --libcamera-control ExposureTime 10000
  ```

- **int64**: 長整数値 ✅

  ```bash
  --libcamera-control FrameDuration 33333
  ```

- **float**: 小数値 ✅

  ```bash
  --libcamera-control AnalogueGain 2.0
  --libcamera-control Brightness -0.5
  ```

- **enum**: 文字列または数値 ✅（主要なenumのみ文字列対応）

  ```bash
  --libcamera-control AfMode Continuous
  # または
  --libcamera-control AfMode 2
  ```

  **文字列対応済みのenum:**
  - `AfMode`, `AfRange`, `AfSpeed`
  - `AeMeteringMode` 
  - `AwbMode`
  - `ExposureTimeMode`, `AnalogueGainMode`

#### 複合型

- **配列**: カンマ区切り ✅

  ```bash
  --libcamera-control ColourGains 1.5,2.0
  --libcamera-control FrameDurationLimits 33333,33333
  ```

  **対応済み:** `float[]`, `int32[]`, `int64[]`

- **矩形（Rectangle）**: x,y,width,height ✅

  ```bash
  --libcamera-control ScalerCrop 100,100,640,480
  ```

- **複数矩形**: セミコロン区切り ✅

  ```bash
  --libcamera-control AfWindows "100,100,200,200;300,300,200,200"
  ```

  **注意:** Rectangle配列の場合、セミコロンで複数の矩形を区切ります

- **マトリクス**: カンマ区切り（行優先） ❌（未対応）

  ```bash
  --libcamera-control ColourCorrectionMatrix 1.0,0,0,0,1.0,0,0,0,1.0
  ```

### 未対応の型

- **Size** 型
- **Point** 型  
- **マトリクス** 型（3x3など）
- その他の複雑な構造体

## 主要なコントロール

### 露出制御（AE: Auto Exposure）

| コントロール | 型 | 説明 | 値の例 |
|------------|---|------|-------|
| AeEnable | bool | 自動露出の有効/無効 | 0, 1 |
| ExposureTime | int32 | 露出時間（マイクロ秒） | 10000 (1/100秒) |
| AnalogueGain | float | アナログゲイン（1.0以上） | 2.0 |
| ExposureTimeMode | enum | 露出時間モード | Auto, Manual |
| AnalogueGainMode | enum | ゲインモード | Auto, Manual |
| AeMeteringMode | enum | 測光モード | CentreWeighted, Spot, Matrix |
| ExposureValue | float | EV補正値 | -2.0 〜 2.0 |

### オートフォーカス（AF）

| コントロール | 型 | 説明 | 値の例 |
|------------|---|------|-------|
| AfMode | enum | AFモード | Manual, Auto, Continuous |
| AfRange | enum | フォーカス範囲 | Normal, Macro, Full |
| AfSpeed | enum | フォーカス速度 | Normal, Fast |
| AfTrigger | enum | AFトリガー | Start, Cancel |
| AfWindows | Rectangle[] | AFエリア | "256,192,512,384" |
| LensPosition | float | レンズ位置（ジオプター） | 2.0 |

### ホワイトバランス（AWB）

| コントロール | 型 | 説明 | 値の例 |
|------------|---|------|-------|
| AwbEnable | bool | AWBの有効/無効 | 0, 1 |
| AwbMode | enum | AWBモード | Auto, Daylight, Cloudy, Tungsten |
| ColourTemperature | int32 | 色温度（ケルビン） | 5500 |
| ColourGains | float[2] | 赤・青ゲイン | 1.5,2.0 |

### 画質調整

| コントロール | 型 | 説明 | 値の例 |
|------------|---|------|-------|
| Brightness | float | 明るさ | -1.0 〜 1.0 |
| Contrast | float | コントラスト | 1.0（標準） |
| Saturation | float | 彩度 | 1.0（標準）, 0.0（モノクロ） |
| Sharpness | float | シャープネス | 0.0 〜 10.0 |
| Gamma | float | ガンマ値 | 2.2（標準） |

### フレームレート制御

| コントロール | 型 | 説明 | 値の例 |
|------------|---|------|-------|
| FrameDurationLimits | int64[2] | フレーム時間の最小/最大（マイクロ秒） | 33333,33333 (30fps固定) |

## enum 値一覧

### AfMode（オートフォーカスモード）

- `Manual` または `0`: 手動フォーカス
- `Auto` または `1`: シングルAF（一度フォーカスして停止）
- `Continuous` または `2`: コンティニュアスAF

### AfRange（フォーカス範囲）

- `Normal` または `0`: 通常範囲
- `Macro` または `1`: マクロ（接写）
- `Full` または `2`: 全範囲

### AfSpeed（フォーカス速度）

- `Normal` または `0`: 通常速度
- `Fast` または `1`: 高速

### ExposureTimeMode / AnalogueGainMode

- `Auto` または `0`: 自動
- `Manual` または `1`: 手動

### AeMeteringMode（測光モード）

- `CentreWeighted` または `0`: 中央重点測光
- `Spot` または `1`: スポット測光
- `Matrix` または `2`: マトリックス測光

### AwbMode（ホワイトバランスモード）

- `Auto` または `0`: 自動
- `Incandescent` または `1`: 白熱灯
- `Tungsten` または `2`: タングステン
- `Fluorescent` または `3`: 蛍光灯
- `Indoor` または `4`: 屋内
- `Daylight` または `5`: 昼光
- `Cloudy` または `6`: 曇天

### HdrMode（HDRモード）

- `Off` または `0`: 無効
- `MultiExposureUnmerged` または `1`: 複数露出（未合成）
- `MultiExposure` または `2`: 複数露出（合成）
- `SingleExposure` または `3`: 単一露出HDR
- `Night` または `4`: ナイトモード

## 使用例

### 例1: 明るい屋外での撮影設定

```bash
./momo --use-libcamera \
       --libcamera-control AeEnable 1 \
       --libcamera-control AeExposureMode Short \
       --libcamera-control AwbMode Daylight \
       --libcamera-control Contrast 1.2 \
       sora ...
```

### 例2: 暗所での撮影設定

```bash
./momo --use-libcamera \
       --libcamera-control ExposureTimeMode Manual \
       --libcamera-control ExposureTime 50000 \
       --libcamera-control AnalogueGain 8.0 \
       --libcamera-control NoiseReductionMode HighQuality \
       sora ...
```

### 例3: マクロ撮影設定

```bash
./momo --use-libcamera \
       --libcamera-control AfMode Continuous \
       --libcamera-control AfRange Macro \
       --libcamera-control AfSpeed Normal \
       --libcamera-control Sharpness 2.0 \
       sora ...
```

### 例4: フレームレート固定（30fps）

```bash
./momo --use-libcamera \
       --libcamera-control FrameDurationLimits 33333,33333 \
       sora ...
```

### 例5: 手動設定での完全制御

```bash
./momo --use-libcamera \
       --libcamera-control AeEnable 0 \
       --libcamera-control AwbEnable 0 \
       --libcamera-control ExposureTime 20000 \
       --libcamera-control AnalogueGain 4.0 \
       --libcamera-control ColourGains 1.8,1.5 \
       --libcamera-control AfMode Manual \
       --libcamera-control LensPosition 2.0 \
       sora ...
```

## トラブルシューティング

### 警告: "Unknown control"

指定したコントロール名が正しくない、またはカメラがそのコントロールをサポートしていません。
利用可能なコントロールはカメラモデルによって異なります。

**動作:** 警告が表示されますが、他の有効な設定は適用されカメラは正常に起動します。

### 警告: "Unsupported control type for"

指定したコントロールの型がサポートされていません（Size、Point、マトリクスなど）。

**動作:** 警告が表示されますが、他の有効な設定は適用されカメラは正常に起動します。

### 警告: "Invalid control value for"

値の形式が正しくないか、範囲外の値を指定しています。

- 数値の範囲を確認してください
- enum の場合は有効な値を確認してください
- 配列の場合は要素数が正しいか確認してください

**動作:** 警告が表示されますが、他の有効な設定は適用されカメラは正常に起動します。

### パフォーマンスの問題

- `--use-libcamera-native` オプションの使用を検討してください（H.264のみ）
- 不要なコントロールの設定を避けてください
- フレームレート制限を適切に設定してください

## コントロールキー一覧

### Core Controls

```
AeConstraintMode
AeEnable
AeExposureMode
AeFlickerDetected
AeFlickerMode
AeFlickerPeriod
AeMeteringMode
AeState
AfMetering
AfMode
AfPause
AfPauseState
AfRange
AfSpeed
AfState
AfTrigger
AfWindows
AnalogueGain
AnalogueGainMode
AwbEnable
AwbLocked
AwbMode
Brightness
ColourCorrectionMatrix
ColourGains
ColourTemperature
Contrast
DebugMetadataEnable
DigitalGain
ExposureTime
ExposureTimeMode
ExposureValue
FocusFoM
FrameDuration
FrameDurationLimits
FrameWallClock
Gamma
HdrChannel
HdrMode
LensPosition
Lux
Saturation
ScalerCrop
SensorBlackLevels
SensorTemperature
SensorTimestamp
Sharpness
```

### Draft Controls

```
AePrecaptureTrigger
AwbState
ColorCorrectionAberrationMode
FaceDetectFaceIds
FaceDetectFaceLandmarks
FaceDetectFaceRectangles
FaceDetectFaceScores
FaceDetectMode
LensShadingMapMode
MaxLatency
NoiseReductionMode
PipelineDepth
SensorRollingShutterSkew
TestPatternMode
```

### Raspberry Pi Specific Controls

```
Bcm2835StatsOutput
CnnEnableInputTensor
CnnInputTensor
CnnInputTensorInfo
CnnKpiInfo
CnnOutputTensor
CnnOutputTensorInfo
PispStatsOutput
ScalerCrops
StatsOutputEnable
SyncFrames
SyncMode
SyncReady
SyncTimer
```

## 参考資料

- <https://libcamera.org/api-html/namespacelibcamera_1_1controls.html>
- <https://github.com/raspberrypi/libcamera/blob/main/src/libcamera/control_ids_core.yaml>
- [libcamera 公式ドキュメント](https://libcamera.org/)
- [Raspberry Pi Camera Algorithm and Tuning Guide](https://datasheets.raspberrypi.com/camera/raspberry-pi-camera-guide.pdf)
