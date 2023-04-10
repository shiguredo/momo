# FAQ

## 商用利用は可能ですか？

Momo のライセンスは [Apache License, Version 2\.0](http://www.apache.org/licenses/LICENSE-2.0) で公開しております。

## コーデックの指定やビットレートを利用できますか？

Momo 側からの指定は WebRTC SFU Sora を利用したときだけ可能です。

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

**是非 Discord の #momo-nvidia-video-codec-sdk チャネルまでご連絡ください**

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

## Sora モードで DataChannel メッセージングは利用できますか？

Momo では Sora の DataChannel メッセージングに対応する予定はありません。 Sora C++ SDK にて対応予定です。

## Raspberry Pi OS のレガシー版には対応していますか？

レガシー版には対応せず、最新版にのみ対応していきます。

## Raspberry Pi (Raspberry-Pi-OS) で `--hw-mjpeg-decoder true` を指定した時に映像が出ません

RaspberryPi の MJPEG デコーダ は一部の MJPEG に対応したカメラでしか機能しません。

MJPEG に対応した CSI カメラや USB カメラをご用意いただくか、 `--hw-mjpeg-decoder false` にしてご利用ください。

## Mac (arm64) から H.264 の FHD でスクリーンキャプチャを配信したい

Mac (arm64) から FHD でスクリーンキャプチャを配信したい場合は Sora の H.264 のプロファイルレベル ID を 3.2 以上に設定してください。 

設定方法はこちらの [Sora のドキュメント](https://sora-doc.shiguredo.jp/sora_conf#default-h264-profile-level-id)をお読みください。

プロファイルレベル ID を変更しない場合は H.264 の HD 以下で配信するか、他のコーデックを使用して FHD 配信をしてください。

## Windows で H.264 を利用できますか？

NVIDIA のビデオカードの NVENC を利用することで H.264 が利用可能になります。
ご利用の環境で H.264 が利用できるかどうかは `./momo --video-codec-engines` を使用して H264 の項目をご確認ください。

## Sora モードを利用するときに Sora の TURN 機能を無効にして利用することはできますか？

Momo は Sora の TURN 機能を無効にして利用することはできません。

## Sora モードで マルチストリーム機能を無効にすることはできますか？

Momo の Sora モードではマルチストリーム機能を無効にして利用することはできません。

## Proxy を設定して利用することはできますか？

以下に示すオプションを指定することで利用可能です。

- `--proxy-url` : プロキシの URL
    - 例) http://proxy.example.com:3128
- `--proxy-username` : プロキシ認証に使用するユーザー名
- `--proxy-password` : プロキシ認証に使用するパスワード

## Raspberry Pi (Raspberry-Pi-OS) 64 bit で H.264 を利用できますか？

利用できません。 Raspberry Pi (Raspberry-Pi-OS) の H.264 は MMAL のハードウェア支援を受けており MMAL が 64 bit に対応していないためです。

## Momo のオプションで指定した解像度で映像の送信はできますか？

`--resolution` オプションを使用することで解像度の設定が可能です。

しかしこれはあくまでも Momo からカメラデバイスに対して送るリクエストであり、最終的な解像度はカメラデバイスの仕様に依存します。

もし、カメラデバイスのサポート外の解像度を試してみたい方は以下の記事を参考にしてみてください。

[WebRTC momoでカメラのサポートしていない解像度、フレームレートで送信する](https://zenn.dev/tetsu_koba/articles/c3b12bb5e52a57)