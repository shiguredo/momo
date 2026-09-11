# video codec エンジン名を sora-cpp-sdk と揃える

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/change-align-video-codec-engine-names
- Polished: {YYYY-MM-DD}
- Reporter: @torikizi

## 目的

Momo の `--video-codec-engines` と `--*-encoder` / `--*-decoder` で使うビデオコーデックエンジンの名前が Sora C++ SDK の `sora::VideoCodecImplementation` の文字列表現と異なる。同じエンジンを指しているのに Momo と Sora C++ SDK で別の名前になるため、両者を併用する利用者が対応関係を把握しづらい。Sora C++ SDK の名前を正として揃える。

## 現状

- `src/video_codec_info.h` の `VideoCodecInfo::TypeToString` が (表示名, オプション値) の組を返す
  - `Type::NVIDIA` -> `{"NVIDIA VIDEO CODEC SDK", "nvidia"}`
  - `Type::Intel` -> `{"Intel VPL", "vpl"}`
  - `Type::V4L2` -> `{"V4L2", "v4l2"}`
  - `Type::Software` -> `{"Software", "software"}`
  - `Type::VideoToolbox` -> `{"VideoToolbox", "videotoolbox"}`
  - `Type::Jetson` -> `{"Jetson", "jetson"}`
- `src/util.cpp` の `Util::ShowVideoCodecs` は `TypeToString` の組を `- {表示名} [{オプション値}]` の形式で出力する
- `src/util.cpp` の `--*-encoder` / `--*-decoder` は `VideoCodecInfo::GetValidMappingInfo` が返す組のオプション値を `CLI::CheckedTransformer` のキーとして受け付ける。`--h264-encoder nvidia` のように指定する
- Sora C++ SDK の `sora::VideoCodecImplementation` は `src/sora_video_codec.cpp` の `tag_invoke` で次の文字列に変換される (`src/sora-cpp-sdk/` には取り込まれていないが、Sora C++ SDK 側で定義されている)
  - `internal` / `cisco_openh264` / `intel_vpl` / `nvidia_video_codec` / `amd_amf` / `raspi_v4l2m2m`
- 対応関係は次のとおり
  - Momo `Type::NVIDIA` (`nvidia`) = Sora C++ SDK `nvidia_video_codec`
  - Momo `Type::Intel` (`vpl`) = Sora C++ SDK `intel_vpl`
  - Momo `Type::V4L2` (`v4l2`) = Sora C++ SDK `raspi_v4l2m2m`
  - Momo `Type::Software` (`software`) は Sora C++ SDK の `internal` に対応するが、Momo は macOS の `Type::VideoToolbox` を別に持つため一対一ではない
  - Sora C++ SDK の `cisco_openh264` / `amd_amf` に対応する名前を Momo は持たない。Momo `Type::Jetson` / `Type::VideoToolbox` に対応する名前を Sora C++ SDK は持たない
- `doc/USE.md` / `doc/VPL.md` / `doc/FAQ.md` に `nvidia` / `vpl` / `v4l2` / `software` が実行例・説明として載っている

## 設計方針

- Sora C++ SDK の `VideoCodecImplementation` の文字列を正とし、`VideoCodecInfo::TypeToString` のオプション値を揃える
  - `Type::NVIDIA` -> `nvidia_video_codec`
  - `Type::Intel` -> `intel_vpl`
  - `Type::V4L2` -> `raspi_v4l2m2m`
- 表示名 (`NVIDIA VIDEO CODEC SDK` / `Intel VPL` / `V4L2`) は利用者がエンジンを判別しやすい現状の表記を維持する。揃えるのは `--video-codec-engines` の `[ ]` 内と CLI オプション値
- CLI オプション値は利用者が `--h264-encoder nvidia` のように指定する公開インターフェースであり、変更は後方互換を壊す。旧名を `CLI::CheckedTransformer` のエイリアスとして受け付けるか、破壊的変更として CHANGES.md に記載するかをここで決める
- `Type::Software` / `Type::VideoToolbox` / `Type::Jetson` は Sora C++ SDK と一対一で対応しないため本 issue では変更しない
- `doc/USE.md` / `doc/VPL.md` / `doc/FAQ.md` の実行例と説明を新しい名前に更新する

## 完了条件

- `--video-codec-engines` の `[ ]` 内と `--*-encoder` / `--*-decoder` に指定できる値が `nvidia_video_codec` / `intel_vpl` / `raspi_v4l2m2m` になる
- 旧名 (`nvidia` / `vpl` / `v4l2`) の互換性の扱いが決まり、CHANGES.md に記載されている
- `doc/USE.md` / `doc/VPL.md` / `doc/FAQ.md` の該当箇所が新名前に更新されている
- NVIDIA / Intel VPL / Raspberry Pi (V4L2 M2M) の各環境で `--video-codec-engines` の出力と `--*-encoder` の指定が一致する (手動確認)

## 解決方法

{YYYY-MM-DD} に追記する
