# Sora C++ SDK から持ってきたソースコードを配置するディレクトリ

## 前提

LAST_UPDATED に記載されている LAST_UPDATED_MOMO と LAST_UPDATED_SORA_CPP_SDK が、お互いに最後に同期した時のコミットハッシュであること。

## スクリプト

- `diff-sora-cpp-sdk.sh <Sora C++ SDK dir> <target_commit>`
    - Sora C++ SDK 側の、最後に同期したコミットから指定されたコミットまでの差分を表示するスクリプト
    - Momo の src/sora-cpp-sdk に存在するファイルに対する差分しか表示しないようになっている
- `diff-momo.sh <target_commit>`
    - Momo 側の、最後に同期したコミットから指定されたコミットまでの差分を表示するスクリプト
    - Momo の src/sora-cpp-sdk に存在するファイルに対する差分しか表示しないようになっている
- `copy-from-sora-cpp-sdk.sh <Sora C++ SDK dir>`
    - Momo の src/sora-cpp-sdk に存在するファイルに対して実際の sora-cpp-sdk のディレクトリからファイルをコピーする
- `copy-to-sora-cpp-sdk.sh <Sora C++ SDK dir>`
    - Momo の src/sora-cpp-sdk に存在するファイルを sora-cpp-sdk のディレクトリにコピーする

## 使い方

1.  `./diff-sora-cpp-sdk.sh ../../../sora-cpp-sdk develop` や `./diff-momo.sh develop` を実行して、差分が小さい方を確認する
2. 仮に `diff-sora-cpp-sdk.sh` の結果の方が小さい場合:
    1. Momo の `src/sora-cpp-sdk` に移動して `./diff-sora-cpp-sdk.sh | patch -p1` でパッチを適用する
    2. コンフリクトが起きたら修正する
    3. `./copy-to-sora-cpp-sdk.sh ../../../sora-cpp-sdk` を実行して Sora C++ SDK にもソースを反映する
3. 仮に `diff-momo.sh` の結果の方が小さい場合:
    1. Sora C++ SDK に移動して `../momo/sora-cpp-sdk/diff-sora-cpp-sdk.sh | patch -p1` でパッチを適用する
    2. コンフリクトが起きたら修正する
    3. `../momo/sora-cpp-sdk/copy-to-sora-cpp-sdk.sh .` を実行して Momo にもソースを反映する
4. Momo と Sora C++ SDK をコミットする
5. `./update-last-updated.sh` を実行して `LAST_UPDATED` のコミットハッシュを更新し、再度コミットする
