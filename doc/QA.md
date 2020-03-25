# Q&A

## 商用利用は可能ですか？

Momo は Apache License 2.0 で公開しております。ライセンスを守ってお使いください。

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
