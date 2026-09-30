# Intel VPL の AV1 デコーダーが新しいメディアスタックで検出されない

- Created: 2026-09-30
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-vpl-av1-decoder-detection
- Polished: {YYYY-MM-DD}

## 目的

Intel VPL の AV1 デコーダーを持つ環境で AV1 デコーダーの検出に失敗し、`--av1-decoder vpl` を指定できない問題を修正する。

## 現状

- `momo --video-codec-engines` で AV1 デコーダーに Intel VPL が現れない環境がある
  - Ubuntu 26.04.1 の self-hosted runner (Intel Core Ultra 5 125H / Meteor Lake、intel-media-va-driver-non-free 26.1.2、libmfx-gen1.2 26.1.2、libvpl2 2.16.0) では AV1 デコーダーが Software のみになる。VP9 / H264 / H265 のデコーダーと VP9 / AV1 / H264 / H265 のエンコーダーは Intel VPL が検出される
  - 同日に実行された Ubuntu 24.04.4 の self-hosted runner では AV1 デコーダーに Intel VPL が検出される
- Intel VPL E2E の `test/test_sora_mode_intel_vpl.py::test_sora_sendonly_recvonly_pair[AV1]` が次のエラーで失敗する (momo の exit code は 105)
  - `--av1-decoder: Check vpl value in {default->0,software->6} OR {0,6} FAILED`
  - 失敗した実行の例: https://github.com/shiguredo/momo/actions/runs/36655931069
- ハードウェアとドライバスタックには AV1 デコードの能力がある
  - `vainfo` は `VAProfileAV1Profile0` の `VAEntrypointVLD` を報告する
  - `vpl-inspect` は `mfx-gen` のデコーダー一覧に `MFX_CODEC_AV1` を列挙する
  - `libvpl-tools` の `sample_encode av1` で作成した IVF を `sample_decode av1` でデコードできる
- 関連する実装
  - `src/video_codec_info.h` の `VideoCodecInfo::GetLinux` が `USE_VPL_ENCODER` 時に `sora::VplVideoDecoder::IsSupported` の結果で `av1_decoders` に `Type::Intel` を追加する。ここで失敗すると `--av1-decoder` の候補から `vpl` が消える
  - `src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_decoder.cpp` の `VplVideoDecoder::IsSupported` と `VplVideoDecoderImpl::CreateDecoderInternal` が検出の本体
  - `CreateDecoderInternal` は AV1 のときだけ `param.mfx.CodecLevel = MFX_LEVEL_AV1_2` を設定する (コメントに「この設定がないと Query しても sts=-3 で失敗する」「MFX_LEVEL_AV1_2 で妥当かどうかはよく分からない」とある)
  - `Query` / `QueryIOSurf` / `Init` のいずれかが失敗すると nullptr を返し、AV1 デコーダーが候補から消える
- どの API がどのステータスで失敗したかはログに出ない。`VideoCodecInfo::Get` は `src/main.cpp` の `Util::ParseArgs` から呼ばれ、`webrtc::LogMessage` の初期化前に実行されるため `LS_VERBOSE` のログが破棄される

## 設計方針

- `VplVideoDecoderImpl::CreateDecoderInternal` の `Query` / `QueryIOSurf` / `Init` のどこがどのステータスで失敗しているかを特定する。`MFX_LEVEL_AV1_2` の指定あり/なし、解像度 (4096x4096 / 2048x2048) の違いで挙動を比較する
- `MFX_LEVEL_AV1_2` の固定が原因の場合は、新しいメディアスタックでも AV1 デコーダーを検出できる設定に変更する。`MFX_LEVEL_AV1_2` が必要だった `sts=-3` の回避と両立させる
- 検出結果がデグレしないことを確認する。Ubuntu 24.04 の runner でも AV1 デコーダーに Intel VPL が出続けること
- 修正は `src/sora-cpp-sdk` 配下に対して行い、upstream の sora-cpp-sdk にも同様の問題がないか確認する
- 0080 で追加する E2E の xfail 回避策は本修正で不要になるため、あわせて削除する

## 完了条件

- Ubuntu 26.04 の self-hosted runner で `momo --video-codec-engines` の AV1 デコーダーに `Intel VPL [vpl]` が出る
- Ubuntu 24.04 の self-hosted runner でも AV1 デコーダーに `Intel VPL [vpl]` が出続ける
- Intel VPL E2E の 12 件が両方の runner で成功する
- `CHANGES.md` の `## develop` に追記する

## 解決方法

{YYYY-MM-DD} に追記する
