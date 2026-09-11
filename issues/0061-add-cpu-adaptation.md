# CPU アダプテーションの有効/無効を設定できるようにする

- Created: 2026-09-10
- Completed: {YYYY-MM-DD}
- Branch: feature/add-cpu-adaptation
- Polished: {YYYY-MM-DD}
- Reporter: @Hexa

## 目的

momo は macOS のサイマルキャスト時に限り CPU アダプテーションを自動で無効にしているが、それ以外の環境では常に有効のままで、アプリから明示的に無効にできない。CPU 負荷が高い状況でも解像度やフレームレートを自動で落とさず、CPU の限界を超えた画質・フレームレートで配信したいという要望がある。

sora-cpp-sdk は 2025.5.0 で `SoraSignalingConfig::cpu_adaptation` (`std::optional<bool>`) を追加し、sumomo では `--cpu-adaptation true|false|none` で指定できる。momo でも同じ設定を指定できるようにする。

参考:

- sora-cpp-sdk 2025.5.0 の `[ADD] CPU アダプテーションの有効/無効を設定できるように `SoraSignalingConfig` に `cpu_adaptation` フィールドを追加`
- sumomo の `--cpu-adaptation` オプション

## 現状

- `src/sora/sora_client.cpp` の `SoraClient::CreateRTCConnection` は macOS のサイマルキャスト時のみ `rtc_config.set_cpu_adaptation(false)` を呼ぶ。それ以外の環境では libwebrtc の既定値 (有効) のまま
- `src/sora/sora_client.h` の `SoraClientConfig` に CPU アダプテーションの設定項目がない
- `src/momo_args.h` の `MomoArgs` と `src/util.cpp` の `Util::ParseArgs` に対応するオプションがない
- `src/util.cpp` の `add_optional_bool` は `--data-channel-signaling` と `--ignore-disconnect-websocket` で `true` / `false` / `none` を扱っている。CPU アダプテーションの指定にも同じ仕組みが使える
- `test/momo.py` は Sora モード専用オプションを列挙しており、オプションを追加するときはテスト用ラッパーの対応も必要
- sora-cpp-sdk 2025.5.0 の `SoraSignalingConfig::cpu_adaptation` と sumomo の `--cpu-adaptation` は実装済みで、momo だけが未対応

## 設計方針

- `MomoArgs` に `boost::optional<bool> sora_cpu_adaptation;` を追加し、`Util::ParseArgs` の Sora モードオプションに `add_optional_bool(sora_app, "--cpu-adaptation", args.sora_cpu_adaptation, "Enable/disable CPU adaptation (default: none)")` を追加する
  - sumomo と同じく `true` / `false` / `none` を受け付ける。`none` は未指定 (従来動作) とする
- `SoraClientConfig` に `boost::optional<bool> cpu_adaptation;` を追加し、`src/main.cpp` で `config.cpu_adaptation = args.sora_cpu_adaptation;` を設定する
- `SoraClient::CreateRTCConnection` で、指定時は `rtc_config.set_cpu_adaptation(*config_.cpu_adaptation)` を呼ぶ。未指定時のみ従来どおり macOS のサイマルキャスト時に `false` にする
  - 明示指定を macOS のサイマルキャスト時の自動無効化より優先する
- `test/momo.py` に `cpu_adaptation` パラメーターと `--cpu-adaptation` の引数変換を追加し、Sora モード専用オプションの一覧にも追加する
- `doc/USE.md` の Sora モードヘルプに `--cpu-adaptation` を追記する
- `CHANGES.md` の `## develop` に `[ADD]` エントリを追記する

## 懸念

- CPU アダプテーションが実際に効いているかどうかはログから判断しづらく、sora-cpp-sdk 側でも検証方法が課題になっている。momo のテストでは設定の指定と接続の成否までを確認する

## テスト方針

- `--cpu-adaptation` を `true` / `false` / `none` で指定して起動できることを確認する
- `--cpu-adaptation` に `true` / `false` / `none` 以外を指定すると CLI のエラーになることを確認する
- `--cpu-adaptation false` を指定して Sora に接続し、映像が送信されることを確認する
- `--cpu-adaptation` 未指定時の macOS サイマルキャストの従来動作 (自動的に無効) が変わらないことを確認する
- `cpu_adaptation` が Sora モード専用オプションとして扱われることを `test/test_momo_validation.py` で確認する
- 既存の Sora モードの E2E テスト (`test/test_sora_mode.py` など) が通ることを確認する

## 完了条件

- `--cpu-adaptation true|false|none` を指定できる
- 明示指定時は macOS サイマルキャスト時の自動無効化より指定値が優先される
- `--cpu-adaptation` 未指定時は従来どおりの動作になる
- `doc/USE.md` の Sora モードヘルプに `--cpu-adaptation` が記載されている
- `CHANGES.md` の `## develop` に `[ADD]` エントリが追記されている
- 既存の Sora モードの E2E テストが通る

## 解決方法

未着手 (PR 作成後に追記する)
