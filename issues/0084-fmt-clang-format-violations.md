# clang-format の違反を修正し、prek のフックをバージョン固定する

- Created: 2026-11-16
- Completed: {YYYY-MM-DD}
- Branch: feature/refactor-clang-format-hook
- Polished: {YYYY-MM-DD}

## 目的

`prek.toml` の `clang-format` フックが `entry = "clang-format -i"` と `language = "system"` のため、フックは開発者の PATH 上にある任意の `clang-format` を実行する。CI は `llvm.sh 20` で clang-format 20 を導入しているが、`prek.toml` はそのバージョンを参照していない。

この状態で `prek run --all-files` を実行すると、未整形の C++ ファイルが書き換えられる。実際にリリース作業中に `prek` が作業対象外の C++ ファイルを変更し、リリース差分に整形差分が混入した。

本 issue では、リポジトリの C++ ソースを固定した 1 バージョンの `clang-format` に適合させ、フックのバージョンを固定して、フックの実行結果が環境に依存しない状態にする。

## 現状

`git ls-files '*.cpp' '*.h' '*.mm'` の 163 ファイルを `clang-format --dry-run` で検査すると、8 ファイルが違反として報告される。

| ファイル | 18.1.8 | 20.1.0 | 23.1.5 |
| --- | --- | --- | --- |
| `src/util.h` | 3 | 3 | 3 |
| `src/ayame/ayame_client.cpp` | 3 | 3 | 3 |
| `src/rtc/fake_audio_capturer.cpp` | 1 | 1 | 1 |
| `src/rtc/rtc_manager.cpp` | 4 | 4 | 6 |
| `src/rtc/rtc_manager.h` | 2 | 2 | 2 |
| `src/sora/sora_client.cpp` | 6 | 6 | 6 |
| `src/sora/sora_data_channel_on_asio.h` | 2 | 2 | 2 |
| `src/sora-cpp-sdk/src/v4l2/v4l2_video_capturer.cpp` | 1 | 1 | 1 |
| 合計 | 22 | 22 | 24 |

違反するファイルの集合は 3 バージョンで共通であり、リポジトリの C++ ソースはいずれのバージョンにも適合していない。

ただし 3 バージョンすべてで違反 0 件になる整形結果は存在しない。`src/rtc/rtc_manager.cpp` の `#if defined(_WIN32)` / `#else` 分岐にある `webrtc::CreateAudioDeviceModule` の呼び出しで、20.1.0 は `#else` 以降の行を 10 スペースに保つが、23.1.5 は 6 スペースへ切り下げる。20.1.0 で整形した結果を 23.1.5 で検査すると 2 件、23.1.5 で整形した結果を 20.1.0 で検査しても 2 件の違反が出る。18.1.8 の整形結果は 8 ファイルについては 20.1.0 と一致する。

違反の内容は次のとおりである。

- 引数の折り返し位置が clang-format の判断と異なるもの。`Websocket` のコンストラクタ、`RTC_LOG` の `<<` 演算子の改行位置、1 行に収まる内容の不要な分割など
- インデントが崩れているもの。`src/sora/sora_data_channel_on_asio.h` の `Close` で `timer_.expires_after(` が 6 スペース、続く引数が 8 スペースになっており、関数本体の 4 スペースと揃っていない。`src/sora-cpp-sdk/src/v4l2/v4l2_video_capturer.cpp` では同じ if ブロック内のコメント 2 行のうち「正確なサイズを得るため EOF を探す」だけインデントが浅い
- `src/util.h` の `MOMO_BOOST_ERROR` マクロがバックスラッシュで 3 行に折り返している。1 行目と 2 行目は 73 桁だが 3 行目は 69 桁で揃っておらず、clang-format は折り返し自体が不要と判断して 2 行に統合する

`.github/workflows/` に `clang-format` や `prek run` を実行するステップは無い。整形の逸脱は CI で検出されない。

## 設計方針

### 正とするバージョン

- `clang-format` 20.1.0 を正とする。CI が導入する `llvm.sh 20` の系統と揃えるためである
- 18.1.8 は 8 ファイルの整形結果が 20.1.0 と一致するため、実質的に互換である
- 23.1.5 は `#if` 分岐のインデント方針が変わるため対象外とする。`prek.toml` でバージョンを固定するため、開発者の PATH 上の 23 系には依存しない

### コードの整形

- 整形のみを行い、コードの動作を変えない。整形前後に、コメントを除いて空白と改行を正規化した文字列が一致することを確認する
- 8 ファイルを `clang-format` 20.1.0 で整形する
- インデント崩れ 2 件は整形で解消する。`Close` の `timer_.expires_after` と `v4l2_video_capturer.cpp` のコメントは、整形結果がそのまま正しいインデントになる
- `src/util.h` の `MOMO_BOOST_ERROR` マクロは整形結果を受け入れ、`// clang-format off` による除外は行わない。このマクロは `__LINE__` を使っておらず `__func__` のみのため、行の結合で動作は変わらない。物理行数が減ることは許容する
- 本 issue では `.clang-format` の設定値を変更しない。`ColumnLimit` などの値を変えると、解決すべきがコード側か設定側かが曖昧になるためである

### prek のフック設定

- 整形コミットとフック設定の変更は分ける。先に整形を完了させ、その状態で `prek run` が通ることを確認してから設定を変える
- `prek.toml` の `clang-format` は `language = "python"` と `additional_dependencies = ["clang-format==20.1.0"]` でバージョンを固定する。PATH 上のバイナリに依存しない
- フックの `entry` は `clang-format -i` ではなく `clang-format --dry-run --Werror` にする。コミット時にファイルを書き換えず、整形されていないことを検出して失敗させる

### 本 issue の範囲外

- CI への `prek run --all-files` の追加は本 issue に含めない。`.github/workflows/build.yml` の変更は別の作業であり、`shiguredo-github-actions` の規約も別途確認が必要になる
- `src/sora-cpp-sdk/` の変更を sora-cpp-sdk リポジトリへ反映する作業は本 issue に含めない。別リポジトリでのコミットとマージが必要になるためである。本 issue では momo 側の整形のみを行い、`LAST_UPDATED` は更新しない

## 完了条件

- clang-format 20.1.0 で 163 ファイルを検査して違反が 0 件である
- clang-format 18.1.8 でも 8 ファイルの違反が 0 件である
- `src/sora/sora_data_channel_on_asio.h` の `Close` で `timer_.expires_after` の行が 4 スペースのインデントになっている
- `src/sora-cpp-sdk/src/v4l2/v4l2_video_capturer.cpp` で、同じ if ブロック内のコメント 2 行のインデントが揃っている
- 整形前後で、コメントを除いて空白と改行を正規化した文字列が 8 ファイルすべてで一致している
- `prek.toml` の `clang-format` フックが `language = "python"` と `additional_dependencies = ["clang-format==20.1.0"]` になり、`entry` が `clang-format --dry-run --Werror` になっている
- PATH から `clang-format` を外した状態でも、clang-format 18.1.8 が PATH にある状態でも、`prek run clang-format --all-files` の終了コードが 0 になり、同じファイル集合が検査される
- `prek run --all-files` を実行しても、整形済みのファイルが書き換えられない

## 解決方法

{YYYY-MM-DD} に追記する
