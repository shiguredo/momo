# Momo E2E テスト

momo の HTTP API を利用した pytest ベースの E2E テストです。

## セットアップ

```bash
# 依存パッケージのインストール
uv sync
```

## ビルド

テストを実行する前に、momo をビルドする必要があります：

```bash
# macOS ARM64 の場合
python3 run.py build macos_arm64

# その他のプラットフォームの場合
python3 run.py build <target>
```

## テスト実行

```bash
# 全てのテストを実行
uv run pytest

# 詳細な出力付きで実行
uv run pytest -v

# 特定のテストを実行
uv run pytest test_momo.py::test_metrics_endpoint_returns_200
```

## 環境変数

- `MOMO_TARGET`: ビルドターゲットを明示的に指定する場合に使用

  ```bash
  MOMO_TARGET=ubuntu-24.04_x86_64 uv run pytest
  ```

## テスト内容

### Test モード

Test モードは追加設定なしで実行できます。以下のテストが含まれます：

- HTTP メトリクスエンドポイント (`/metrics`) のテスト
  - ステータスコードの確認
  - JSON レスポンスの構造確認
  - W3C WebRTC 統計情報のフォーマット確認
  - エラー処理の確認

### Sora モード

Sora モードのテストを実行するには、`.env` ファイルの設定が必要です。

#### セットアップ

1. `.env.template` を `.env` にコピー

   ```bash
   cp .env.template .env
   ```

2. `.env` ファイルを編集して、実際の Sora サーバー情報を設定

   ```bash
   TEST_SORA_SIGNALING_URLS=wss://your-sora-server.com/signaling
   TEST_SORA_CHANNEL_ID_PREFIX=test_
   ```

   **注意**:
   - チャンネル ID は自動的に生成されます（プレフィックス + UUID）
   - 複数の Signaling URL を指定する場合はカンマ区切りで記述可能

3. Sora モードのテストを実行

   ```bash
   uv run pytest test_sora.py -v
   ```

#### Sora モードの動作

- `--fake-capture-device` オプションを使用して仮想的なオーディオ・ビデオストリームを送信
- `--role sendonly` で送信のみモードで起動
- `--audio true --video true` でメディアストリームを有効化
- メトリクスサーバーはポート 8081 で起動

#### オプション環境変数

必要に応じて以下の環境変数も設定可能：

- `TEST_SORA_METADATA`: JSON 形式のメタデータ
- `TEST_SORA_CLIENT_ID`: クライアント ID
- `TEST_SORA_BUNDLE_ID`: バンドル ID

**注意**: Sora モードのテストは、環境変数 `TEST_SORA_SIGNALING_URLS` が設定されていない場合は自動的にスキップされます。

## ファイル構成

```plaintext
e2e-test/
├── .env.template           # 環境変数のテンプレート
├── .gitignore             # Git 除外設定
├── conftest.py            # pytest 設定（HTTP クライアント）
├── momo.py                # Momo プロセス管理クラス
├── pyproject.toml         # プロジェクト設定と依存関係
├── test_metrics_api.py     # メトリクス API のテスト
├── test_p2p_mode.py       # P2P (Test) モードのテスト
├── test_momo_validation.py # モード固有オプション検証のテスト
├── test_sora_mode.py      # Sora モードのテスト
└── README.md              # このファイル
```

## トラブルシューティング

### momo が起動しない場合

- ビルドが完了していることを確認
- `_build/<target>/release/momo/momo` に実行ファイルが存在することを確認
- ポート 8080、8081 が他のプロセスで使用されていないことを確認

### Sora モードのテストが失敗する場合

- Signaling URL が正しいことを確認
- ネットワーク接続を確認
- Sora サーバーが稼働していることを確認
