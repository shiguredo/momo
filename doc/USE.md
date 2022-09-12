# Momo を使ってみる

## 準備

### NVIDIA Jetson シリーズで Momo を準備する

[SETUP_JETSON.md](SETUP_JETSON.md) をお読みください。

### Raspberry Pi で Momo を 準備する

[SETUP_RASPBERRY_PI.md](SETUP_RASPBERRY_PI.md) をお読みください。

### macOS で Momo を準備する

[SETUP_MAC.md](SETUP_MAC.md) をお読みください。

### Windows で Momo を準備する

[SETUP_WINDOWS.md](SETUP_WINDOWS.md) をお読みください。

## 動かす

### テストモードを利用して Momo を動かしてみる

[USE_TEST.md](USE_TEST.md) をお読みください。

### Ayame モードを 利用して Momo を動かしてみる

Ayame モードでは時雨堂が開発しているオープンソースのシグナリングサーバ [WebRTC Signaling Server Ayame](https://github.com/OpenAyame/ayame) を利用します。

[Ayame Labo](https://ayame-labo.shiguredo.jp/) を利用することで、 Ayame を用意することなく Ayame を試すことが可能です。

[USE_AYAME.md](USE_AYAME.md) をお読みください。

### Sora モードを 利用して Momo を動かしてみる

Sora モードでは時雨堂が開発、販売している WebRTC SFU Sora を利用します。

[Sora Labo](https://sora-labo.shiguredo.jp/) を利用することで、 Sora を無料で試すことが可能です。

[USE_SORA.md](USE_SORA.md) をお読みください。

### データチャネルを利用したシリアル読み書きを使ってみる

Test と Ayame モードではデータチャネルを利用して指定したシリアルポートに対して送受信が可能です。

[USE_SERIAL.md](USE_SERIAL.md) をお読みください。

### SDL を利用した受信機能を使ってみる

Momo では SDL (Simple DirectMedia Layer) を利用して音声や映像を出力することが可能になります。

[USE_SDL.md](USE_SDL.md) をお読みください。

### Momo の統計情報を取得する

Momo では統計情報を見るようにメトリクス API を用意しています。

[USE_METRICS.md](USE_METRICS.md) をお読みください。

## FAQ

FAQ に関しては [FAQ.md](FAQ.md) をお読みください。

## コマンド

### バージョン情報

```
$ ./momo --version
WebRTC Native Client Momo 2022.2.0 (8b57be45)

WebRTC: Shiguredo-Build M102.5005@{#7} (102.5005.7.4 6ff73180)
Environment: [arm64] macOS Version 12.3 (Build 21E230)

USE_MMAL_ENCODER=0
USE_JETSON_ENCODER=0
USE_NVCODEC_ENCODER=0
USE_SDL2=1
```

### ヘルプ

```
$ ./momo --help
Momo - WebRTC Native Client
Usage: ./momo [OPTIONS] [SUBCOMMAND]

Options:
  -h,--help                   Print this help message and exit
  --help-all                  Print help message for all modes and exit
  --no-google-stun            Do not use google stun
  --no-video-device           Do not use video device
  --no-audio-device           Do not use audio device
  --force-i420                Prefer I420 format for video capture (only on supported devices)
  --hw-mjpeg-decoder          Perform MJPEG deoode and video resize by hardware acceleration (only on supported devices)
  --video-device TEXT         Use the video device specified by an index or a name (use the first one if not specified)
  --resolution TEXT           Video resolution (one of QVGA, VGA, HD, FHD, 4K, or [WIDTH]x[HEIGHT])
  --framerate INT:INT in [1 - 60]
                              Video framerate
  --fixed-resolution          Maintain video resolution in degradation
  --priority TEXT:{BALANCE,FRAMERATE,RESOLUTION}
                              Preference in video degradation (experimental)
  --use-sdl                   Show video using SDL (if SDL is available)
  --show-me                   Show self video (if SDL is available)
  --window-width INT:INT in [180 - 16384]
                              Window width for videos (if SDL is available)
  --window-height INT:INT in [180 - 16384]
                              Window height for videos (if SDL is available)
  --fullscreen                Use fullscreen window for videos (if SDL is available)
  --version                   Show version information
  --insecure                  Allow insecure server connections when using SSL
  --log-level INT:value in {verbose->0,info->1,warning->2,error->3,none->4} OR {0,1,2,3,4}
                              Log severity level threshold
  --screen-capture            Capture screen
  --disable-echo-cancellation Disable echo cancellation for audio
  --disable-auto-gain-control Disable auto gain control for audio
  --disable-noise-suppression Disable noise suppression for audio
  --disable-highpass-filter   Disable highpass filter for audio
  --disable-typing-detection  Disable typing detection for audio
  --video-codec-engines       List available video encoders/decoders
  --vp8-encoder :value in {default->0,software->6} OR {0,6}
                              VP8 Encoder
  --vp8-decoder :value in {default->0,software->6} OR {0,6}
                              VP8 Decoder
  --vp9-encoder :value in {default->0,software->6} OR {0,6}
                              VP9 Encoder
  --vp9-decoder :value in {default->0,software->6} OR {0,6}
                              VP9 Decoder
  --av1-encoder :value in {default->0,software->6} OR {0,6}
                              AV1 Encoder
  --av1-decoder :value in {default->0,software->6} OR {0,6}
                              AV1 Decoder
  --h264-encoder :value in {default->0,videotoolbox->5} OR {0,5}
                              H.264 Encoder
  --h264-decoder :value in {default->0,videotoolbox->5} OR {0,5}
                              H.264 Decoder
  --serial TEXT:serial setting format
                              Serial port settings for datachannel passthrough [DEVICE],[BAUDRATE]
  --metrics-port INT:INT in [-1 - 65535]
                              Metrics server port number (default: -1)
  --metrics-allow-external-ip Allow access to Metrics server from external IP
  --client-cert TEXT:FILE     Cert file path for client certification (PEM format)
  --client-key TEXT:FILE      Private key file path for client certification (PEM format)
  --proxy-url TEXT            Proxy URL
  --proxy-username TEXT       Proxy username
  --proxy-password TEXT       Proxy password

Subcommands:
  test                        Mode for momo development with simple HTTP server
  ayame                       Mode for working with WebRTC Signaling Server Ayame
  sora                        Mode for working with WebRTC SFU Sora
```

#### ビデオコーデックエンジン

```
$ ./momo --video-codec-engines
VP8:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

VP9:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

AV1:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

H264:
  Encoder:
    - VideoToolbox [videotoolbox] (default)
  Decoder:
    - VideoToolbox [videotoolbox] (default)
```

### test モードヘルプ


```
$ ./momo test --help
Mode for momo development with simple HTTP server
Usage: ./momo test [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --help-all                  Print help message for all modes and exit
  --document-root TEXT:DIR    HTTP document root directory
  --port INT:INT in [0 - 65535]
                              Port number (default: 8080)
```

### ayame モードヘルプ


```
$ ./momo ayame --help
Mode for working with WebRTC Signaling Server Ayame
Usage: ./momo ayame [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --help-all                  Print help message for all modes and exit
  --signaling-url TEXT REQUIRED
                              Signaling URL
  --channel-id TEXT REQUIRED  Channel ID
  --client-id TEXT            Client ID
  --signaling-key TEXT        Signaling key
```

### sora モードヘルプ

```
$ ./momo sora --help
Mode for working with WebRTC SFU Sora
Usage: ./momo sora [OPTIONS] SIGNALING-URL CHANNEL-ID

Positionals:
  SIGNALING-URL TEXT REQUIRED Signaling URL
  CHANNEL-ID TEXT REQUIRED    Channel ID

Options:
  -h,--help                   Print this help message and exit
  --help-all                  Print help message for all modes and exit
  --auto                      Connect to Sora automatically
  --video BOOLEAN:value in {false->0,true->1} OR {0,1}
                              Send video to sora (default: true)
  --audio BOOLEAN:value in {false->0,true->1} OR {0,1}
                              Send audio to sora (default: true)
  --video-codec-type TEXT:{,AV1,H264,VP8,VP9}
                              Video codec for send
  --audio-codec-type TEXT:{,OPUS}
  --video-bit-rate INT:INT in [0 - 30000]
                              Video bit rate
  --audio-bit-rate INT:INT in [0 - 510]
                              Audio bit rate
  --role TEXT:{sendonly,recvonly,sendrecv}
                              Role (default: sendonly)
  --spotlight BOOLEAN:value in {false->0,true->1} OR {0,1}
                              Use spotlight
  --spotlight-number INT:INT in [0 - 8]
                              Stream count delivered in spotlight
  --port INT:INT in [-1 - 65535]
                              Port number (default: -1)
  --simulcast BOOLEAN:value in {false->0,true->1} OR {0,1}
                              Use simulcast (default: false)
  --data-channel-signaling TEXT:value in {false-> 0,true-> 1,none->--} OR { 0, 1,--}
                              Use DataChannel for Sora signaling (default: none)
  --data-channel-signaling-timeout INT:POSITIVE
                              Timeout for Data Channel in seconds (default: 180)
  --ignore-disconnect-websocket TEXT:value in {false-> 0,true-> 1,none->--} OR { 0, 1,--}
                              Ignore WebSocket disconnection if using Data Channel (default: none)
  --disconnect-wait-timeout INT:POSITIVE
                              Disconnecting timeout for Data Channel in seconds (default: 5)
  --metadata TEXT:JSON Value  Signaling metadata used in connect message
```

## うまく動作しない時

- カメラを uv4l など他のプロセスが利用していないか確認してください
