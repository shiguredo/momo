# FAQ

## 商用利用は可能ですか？

Momo のライセンスは [Apache License, Version 2\.0](http://www.apache.org/licenses/LICENSE-2.0) で公開しております。

## サイマルキャストは利用できますか？

Sora モードでのみ利用可能です。利用可能な映像コーデックは VP8 と H.264 です。
利用する場合は `--simulcast` を指定してください。

## AV1 は利用できますか？

利用可能です。

## データチャネルは利用できますか？

シリアル経由でのみデータチャネルが利用可能です。

[USE_SERIAL.md](USE_SERIAL.md)

## 認証局の証明書を追加できますか？

`SSL_CERT_DIR` または `SSL_CERT_FILE` 環境変数に CA 証明書のパスを指定することで、サーバ証明書の検証に利用するための CA 証明書を追加することが可能です。

```
$ export SSL_CERT_FILE=/path/to/cert.pem
$ ./momo sora ...
```

## サーバ証明書を無視できますか？

`--insecure` オプションを指定することで、クライアント側でサーバ証明書の検証を行わないようにすることが可能です。

```
$ ./momo --insecure sora ...
```

## NVIDIA ビデオカードに搭載されている NVENC を利用できますか？

Windows と Linux で利用可能です。
NVIDIA ビデオカードドライバーは最新版にしてください。

NVENC が利用可能なビデオカードは以下で確認してください。

[Video Encode and Decode GPU Support Matrix \| NVIDIA Developer](https://developer.nvidia.com/video-encode-decode-gpu-support-matrix#Encoder)

### 動作確認が取れたビデオカード

**是非 Discord の #nvidia-video-codec-sdk チャネルまでご連絡ください**

- GeForce GTX 1080 Ti
    - @melpon
- GeForce GTX 1650
    - @melpon
- GeForce GTX 1060 6GB
    - @massie_g
- GeForce GTX 1050 Ti 4GB
    - @cli_nil
- GeForce GTX 1070 with Max-Q Design 8GB
    - @torikizi
- Quadro P1000 4GB
    - 株式会社オプティム
- Quadro P4000
    - 株式会社オプティム
- GeForce RTX 2070
    - @msnoigrs
- GeForce RTX 2080
    - @shirokunet

## 4K カメラのオススメはありますか？

以下の記事を参考にしてみてください。

[4K webcam について \- Qiita](https://qiita.com/tetsu_koba/items/8b4921f257a46a15d2a7)

## Momo はカメラからの映像以外を入力できますか？

以下の記事を参考にしてみてください。

[カメラ無しのラズパイとmomoでテスト映像をWebRTCで配信する \- Qiita](https://qiita.com/tetsu_koba/items/789a19cb575953f41a1a)

## Momo はマイクからの音声以外を入力できますか？

以下の記事を参考にしてみてください。

[ラズパイ\+momoでWebRTCで送信するときにマイクの代わりに音声ファイルを使用する \- Qiita](https://qiita.com/tetsu_koba/items/b887c1a0be9f26b795f2)

## Mac で 60fps を利用できますか？

利用できません。60fps を使ってみたい場合は Jetson Nano などをご利用ください。

## Raspberry Pi (Raspberry-Pi-OS) で `--hw-mjpeg-decoder true` を指定した時に映像が出ません

RaspberryPi の MJPEG デコーダ は一部の MJPEG に対応したカメラでしか機能しません。

MJPEG に対応した CSI カメラや USB カメラをご用意いただくか、 `--hw-mjpeg-decoder false` にしてご利用ください。
