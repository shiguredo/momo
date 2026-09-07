# Momo の統計情報を取得する

## 概要

Momo には統計情報を HTTP API 経由で JSON 形式で取得することができる MetricsServer が内臓されています。ここでは、MetricsServer の起動方法、レスポンスの形式などについて説明します。

## MetricsServer の起動方法

デフォルトでは MetricsServer は起動しません。起動させたい場合は、`--metrics-port` オプションでポート番号を指定してください。8081 番ポートで起動させたい場合は、次のように指定してください。

```bash
./momo --metrics-port 8081 <mode>
```

MetricsServer はデフォルトでループバック (127.0.0.1 で listen) アドレスからのみアクセス可能です。グローバル (0.0.0.0 で listen) アクセスを許可する場合は `--metrics-allow-external-ip` 引数を指定してください。

## 統計情報 API

### URL

統計情報を取得するには HTTP の GET メソッドを利用して `/metrics` にアクセスしてください。例えば、次のオプションで Momo を起動した場合、

```bash
./momo --metrics-port 8081 p2p
```

<http://127.0.0.1:8081/metrics> にブラウザ、または `curl` などの HTTP クライアントでアクセスすることで統計情報を取得することができます。

### 統計情報 JSON の仕様

統計情報は JSON 形式で取得することができます。JSON に含まれる内容は次の通りです。

```json
{
  "version": "MomoVersion::GetClientName() の戻り値",
  "environment": "MomoVersion::GetEnvironmentName() の戻り値",
  "libwebrtc": "MomoVersion::GetLibwebrtcName() の戻り値",
  "stats": []
}
```

`"stats"` は `webrtc::RTCStats` の配列です。Sora モードの pong メッセージに含まれるものと同じです。

実際の値は `/metrics` の応答で確認してください。`"stats"` の各オブジェクトの形は接続状態によって変わります。

## WebRTC 統計情報の仕様

統計情報 API のレスポンスに含まれる `stats` フィールドの詳細は W3C の標準仕様 [Identifiers for WebRTC's Statistics API](https://www.w3.org/TR/webrtc-stats/) を参考にしてください。

## 応用例

- [自宅の Jetson で動いている WebRTC Native Client Momo を外出先でいい感じに監視する方法](https://zenn.dev/hakobera/articles/c0553faa1223324d6aff)
