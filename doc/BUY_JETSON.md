# Jetson シリーズを購入する

NVIDIA Jetson を購入する場合のオススメリンクです。

## Momo の Jetson シリーズへの対応について

2022 年 10 月時点での状況。

- Jetson シリーズは Jetson AGX Orin を除いて入手困難または異常に高価な販売となっている
- Jetpack 5 系は Jetson nano には対応しない
- Jetson Orin nano はハードウェアエンコードに対応しない
- Jetson Xavier NX は Jetson Nano と比較すると JPEG のハードウェア処理が劣る
- Jetson nano で利用されている Ubuntu 18.04 は 2023 年 4 月でサポートが終了する
    - ただし Jetson nano 自体サポートは 2025 年 1 月までサポート

以上を踏まえて Momo では今後以下のような方針をとることにしました。

- 2023 年 4 月をもって Jetpack 4 系への対応を終了する
    - Jetpack 5 系のみの対応とする
    - それに伴い Jetpack 4 系のみに対応している Jetson Nano の対応を終了する
- Jetson Orin nano への対応は行わない
- Jetson Orin NX への対応は優先実装とする
    - Xavier と同じパターンだと Jetson Orin Nano より JPEG のハードウェア処理が劣る可能性があるため
- Jetson Nano は support/jetpack-4.6 として 2023 年 4 月まで維持する

## Jetson AGX Orin

**以下で買われることをオススメします**

- [NVIDIA Jetson AGX Orin 開発者キット 商品・個数選択](https://ryoyo-direct.jp/shopping/jetson-orin/jetson-orin)

## 4K@30 出るカメラの購入する

実際に Jetson Nano で検証して 4K で 30fps の出力動作確認が取れているカメラです。

- [高解像度 4 18K カメラモジュール 3840 × 2160 ソニー IMX317 Mjpeg 30fps ミニ Usb ウェブカメラ Web カメラ](https://ja.aliexpress.com/item/32999909513.html)

以下は上のタイプのケースありやレンズが色々選べるタイプです。

- https://ja.aliexpress.com/item/33013268769.html
- https://ja.aliexpress.com/item/33016603918.html
- https://ja.aliexpress.com/item/33012473257.html

色々 4K で 30fps が出せるカメラを試してきましたが、このカメラが一番安定しています。

以下が購入の参考になると思います。

[4K webcam について \- Qiita](https://qiita.com/tetsu_koba/items/8b4921f257a46a15d2a7)


## Jetson 関連リンク

- Jetson ロードマップ
    - [Jetson Roadmap \| NVIDIA Developer](https://developer.nvidia.com/embedded/develop/roadmap)
- Jetson ライフサイクル
    - [Jetson Product Lifecycle \| NVIDIA Developer](https://developer.nvidia.com/embedded/lifecycle)

