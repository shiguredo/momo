# libcamerac がローカル配列の `Span` を `ControlValue` に渡しダングリングする

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-libcamerac-span-dangling
- Polished: {YYYY-MM-DD}

## 目的

libcamera の C ラッパが配列コントロールを設定するとき、関数ローカルの `std::vector` から `libcamera::Span` を作り `ControlValue::set` に渡す。`set` が要素をコピーせず Span のポインタだけ持つと、関数終了後にダングリング参照になる。これを修正する。

## 現状

- `src/sora-cpp-sdk/third_party/libcamerac/libcamerac.cpp` の配列コントロール設定が `ControlTypeInteger32` / `Integer64` / `Float` でローカル `values` を作り、`value.set(libcamera::Span<const T>(values.data(), values.size()))` する
- `ControlTypeRectangle` の配列も同様にローカル領域への Span を渡す
- `0023` は `LibcameraCapturer::StopCapture` と request の use-after-free であり、本指摘はコントロール値の Span 寿命である

## 設計方針

- `ControlValue` が所有するコピーへ値を入れる。Span がローカルバッファを指したまま関数を抜けないようにする
- libcamera の `ControlValue::set(Span)` がコピーするなら、コピー後に vector を破棄してよいことをコード上で保証する (一時 vector を set の引数寿命に閉じる、または明示コピー)
- コピーしない API なら `ControlList` 投入まで vector を保持する
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- 配列コントロール設定後にローカル vector を破棄しても、設定値が壊れない
- スカラーコントロールの既存経路は従来通り動作する
- 不正フォーマット時のエラー戻り値は維持する

## 解決方法

未着手 (PR 作成後に追記する)
