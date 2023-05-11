# WebRTC Native Client Momo

[![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/shiguredo/momo.svg)](https://github.com/shiguredo/momo)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Actions Status](https://github.com/shiguredo/momo/workflows/daily-build-workflow/badge.svg)](https://github.com/shiguredo/momo/actions)

## About Shiguredo's open source software

We will not respond to PRs or issues that have not been discussed on Discord. Also, Discord is only available in Japanese.

Please read https://github.com/shiguredo/oss before use.

## About WebRTC Native Client Momo

Momo is a WebRTC Native Client that uses "libwebrtc" and runs on various browser-less devices.
When Momo runs on Jetson Xavier NX / AGX, it is possible to deliver 4K video compressed with H.264 to the browser within 1 second.

## Binary for Jetson Xavier NX / AGX

Download the latest version of the binary for Jetson Xavier NX / AGX from below.
https://github.com/shiguredo/momo/releases

```
momo-<VERSION>_ubuntu-20.04_armv8_jetson_xavier.tar.gz
```

## Use "test mode"

First, try "test mode", where Momo itself has a function as a signaling server.

```shell
$ ./momo --no-audio-device --port 8080 test
```

If momo's IP address is 192.0.2.100, access the following URL and try to connect from WebRTC-compatible browser.

http://192.0.2.100:8080/html/test

## Use "Ayame mode"

"Ayame Labo" is a free signaling server service.

The room ID is "open-momo-en" in the sample, but it is recommended to use an ID that is difficult to guess.

```shell
$ ./momo --no-audio-device ayame wss://ayame-labo.shiguredo.jp/signaling open-momo-en
```

Check with the online sample of Ayame Web SDK.
Access by specifying the room ID in the URL argument.

```
https://openayame.github.io/ayame-web-sdk-samples/recvonly.html?roomId=open-momo-en
```
