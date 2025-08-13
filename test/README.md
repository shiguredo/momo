# Momo E2E テスト

momo の HTTP API を利用した pytest ベースの E2E テストです。

## セットアップ

```bash
# 依存パッケージのインストール
uv sync
```

## フォーマッター

```bash
uv run ruff format
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

## ビルドターゲットの自動検出

テスト実行時、`_build` ディレクトリ内のビルド済みターゲットが自動的に検出されます。

### 検出ロジック

1. **単一ビルドの場合**  
   `_build` ディレクトリに 1 つだけビルドが存在する場合、そのビルドを自動的に選択します。

2. **複数ビルドの場合**  
   実行環境のプラットフォーム（OS）とアーキテクチャ（CPU）を判定し、以下の優先順位で適切なビルドを選択します：

   **macOS（Darwin）**
   - ARM64（Apple Silicon）環境：
     1. `macos_arm64`
     2. `macos_x86_64`（Rosetta 2 での実行用）
   - x86_64（Intel）環境：
     1. `macos_x86_64`
     2. `macos_arm64`

   **Linux**
   - ARM64（aarch64）環境：
     1. `ubuntu-24.04_armv8`
     2. `ubuntu-22.04_armv8`
     3. `ubuntu-20.04_armv8`
   - x86_64 環境：
     1. `ubuntu-24.04_x86_64`
     2. `ubuntu-22.04_x86_64`
     3. `ubuntu-20.04_x86_64`

3. **フォールバック**  
   優先順位リストにマッチするビルドが見つからない場合、利用可能な最初のビルドを使用します。

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
   TEST_SORA_MODE_SIGNALING_URLS=wss://your-sora-server.com/signaling
   TEST_SORA_MODE_CHANNEL_ID_PREFIX=test_
   TEST_SORA_MODE_SECRET_KEY=your_secret_key
   ```

   > [!NOTE]
   > - チャンネル ID は自動的に生成されます（プレフィックス + UUID）
   > - 複数の Signaling URL を指定する場合はカンマ区切りで記述可能

3. Sora モードのテストを実行

   ```bash
   uv run pytest test_sora.py -v
   ```

#### Sora モードの動作

- `--fake-capture-device` オプションを使用して仮想的なオーディオ・ビデオストリームを送信
- `--role sendonly` で送信のみモードで起動
- `--audio true --video true` でメディアストリームを有効化
- メトリクスサーバーはポート 8081 で起動

> [!IMPORTANT]
> Sora モードのテストは、環境変数 `TEST_SORA_MODE_SIGNALING_URLS` が設定されていない場合は自動的にスキップされます。

## ファイル構成

```plaintext
test/
├── .env.template           # 環境変数のテンプレート
├── .gitignore             # Git 除外設定
├── .python-version        # Python バージョン指定
├── conftest.py            # pytest 設定（HTTP クライアント）
├── momo.py                # Momo プロセス管理クラス
├── pyproject.toml         # プロジェクト設定と依存関係
├── test_metrics_api.py     # メトリクス API のテスト
├── test_p2p_mode.py       # P2P (Test) モードのテスト
├── test_momo_validation.py # モード固有オプション検証のテスト
├── test_sora_mode.py      # Sora モードのテスト
├── uv.lock                # uv のロックファイル
└── README.md              # このファイル
```

## GitHub Actions での自動テスト

E2E テストは GitHub Actions で自動実行されます（`.github/workflows/e2e-test.yml`）。

### 実行トリガー

1. **プッシュ時の自動実行**
   - `test/**/*.py` または `.github/workflows/e2e-test.yml` が変更された場合
   - プッシュされたブランチの最新の成功したビルドを自動的に取得してテスト

2. **手動実行（workflow_dispatch）**
   - GitHub Actions の UI から手動でトリガー可能
   - ビルドアーティファクトの URL を指定してテスト実行
   - 例：`https://github.com/shiguredo/momo/actions/runs/{run_id}/artifacts/{artifact_id}`

3. **他のワークフローからの呼び出し（workflow_call）**
   - ビルドワークフローから自動的に呼び出される

### テスト環境

- **対象プラットフォーム**：
  - Ubuntu 22.04 x86_64
  - Ubuntu 24.04 x86_64

- **環境変数（GitHub Secrets）**：
  - `TEST_SIGNALING_URLS`：Sora のシグナリング URL
  - `TEST_CHANNEL_ID_PREFIX`：チャンネル ID のプレフィックス
  - `TEST_SECRET_KEY`：認証用のシークレットキー

### アーティファクトの自動取得

- プッシュ時：そのブランチの最新の成功したビルドを自動検出
- 手動実行時：指定された URL からアーティファクトをダウンロード
- workflow_call 時：前のステップで生成されたアーティファクトを使用

## トラブルシューティング

### momo が起動しない場合

- ビルドが完了していることを確認
- `_build/<target>/release/momo/momo` に実行ファイルが存在することを確認
- ポート 8080、8081 が他のプロセスで使用されていないことを確認

### Sora モードのテストが失敗する場合

- Signaling URL が正しいことを確認
- ネットワーク接続を確認
- Sora サーバーが稼働していることを確認
