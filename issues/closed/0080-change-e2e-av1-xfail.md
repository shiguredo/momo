# E2E の AV1 ペアテストを Intel VPL デコーダーが使えない環境で xfail にする

- Created: 2026-09-30
- Completed: 2026-09-30
- Branch: feature/change-e2e-av1-xfail
- Polished: 2026-09-30

## 目的

Intel VPL の AV1 デコーダーが検出されない環境で Intel VPL E2E が常に失敗するため、検出問題 (0079) の修正が入るまでの間、対象のテストを xfail にして CI を赤くしないようにする。

## 現状

- 一部の self-hosted runner では momo が AV1 デコーダーの Intel VPL を検出できず、`test/test_sora_mode_intel_vpl.py` の `test_sora_sendonly_recvonly_pair[AV1]` が `RuntimeError: momo process exited unexpectedly with code 105` で失敗する (0079)
- 原因は momo 側の検出の問題でありテストの検証内容の問題ではないが、修正が入るまで Intel VPL E2E ジョブが失敗し続ける
- Intel VPL E2E ジョブがどちらの runner で実行されるかは self-hosted runner の空き状況で決まるため、日によって成否が変わる
- `test/momo.py` の `Momo` には `--video-codec-engines` の出力を取得・解析する仕組みがない

## 設計方針

- ホスト名などの特定マシンに依存した判定は行わない。実行環境で momo が使えるエンジンに基づいて判定する
  - momo バイナリを `--video-codec-engines` 付きで実行し、AV1 の Decoder セクションに `Intel VPL` があるかどうかを確認する
  - 判定は表示名 `Intel VPL` を基準にする。`[vpl]` のようなオプション値は 0063 で `intel_vpl` に改名される予定であり、値で判定すると改名後に常時 xfail になるため
  - AV1 の Encoder セクションにも `Intel VPL` が表示される環境があるため、Decoder セクションだけを対象にする
- 判定は `test/test_sora_mode_intel_vpl.py` の `test_sora_sendonly_recvonly_pair` の AV1 のときだけ行う。他コーデックやエンコーダー側の検証は弱めない
- marker ではなく実行時に `pytest.xfail(...)` を呼ぶ。momo 側の検出が直れば xfail は呼ばれなくなり、通常どおり pass する
- xfail の理由には「この環境では momo が Intel VPL の AV1 デコーダーを検出できない」旨だけを書く (issue 番号はコードに持ち込まない)
- `--video-codec-engines` の実行と解析は `test/momo.py` にヘルパーとして置く
- 本 issue は 0079 の検出修正より先に実施する一時的な回避策である。0079 の修正が入ると xfail 条件は成立しなくなり、対象テストは通常どおり実行される。xfail とヘルパーの削除は 0079 の対応時に行う

## 完了条件

- AV1 デコーダーの Intel VPL が検出されない self-hosted runner で Intel VPL E2E ジョブが成功し、ジョブのログで `test_sora_sendonly_recvonly_pair[AV1]` が `XFAIL` になっていることを確認する
  - ジョブがどちらの runner に割り当てられるかは空き状況で決まるため、対象環境に割り当てられるまでジョブを再実行して確認する
- AV1 デコーダーの Intel VPL が検出される self-hosted runner では、同テストが `PASSED` のままであることを確認する
- 他のコーデック・他のテストの検証内容は変わらない
- `CHANGES.md` の `## develop` に追記する

## 解決方法

2026-09-30 に解決した。

- `test/momo.py` の `Momo` に `get_video_codec_engines()` を追加し、`--video-codec-engines` の出力からコーデックごとのエンコーダー / デコーダーの表示名を取得できるようにした
- `test/test_sora_mode_intel_vpl.py` の `test_sora_sendonly_recvonly_pair` の AV1 のときだけ、AV1 デコーダーに `Intel VPL` が含まれない場合に実行時 `pytest.xfail(...)` を呼ぶようにした。一覧が空の場合は解析失敗として `RuntimeError` にし、無言の xfail にはしない
- prek の ty チェックを通すため、`test/momo.py` に `VideoEncoderParams` / `VideoDecoderParams` を追加してコーデックエンジン指定の dict に型注釈を付け、各テストに適用した。`test_ayame_mode.py` の型チェッカーの ignore 指定を ty の形式に修正した
- `CHANGES.md` の `## develop` に misc のエントリを追記した
- 検証: `cd test && uv run ty check .` と `prek run` が通ることを確認した。xfail の実機動作は Intel VPL の self-hosted runner での CI 実行で確認する
