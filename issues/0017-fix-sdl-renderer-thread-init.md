# SDL 初期化失敗時に未初期化のスレッドハンドルを SDL_WaitThread に渡してクラッシュする

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-sdl-renderer-thread-init
- Polished: {YYYY-MM-DD}

## 目的

`SDLRenderer` のコンストラクタは `SDL_Init` / `SDL_CreateWindow` / `SDL_CreateRenderer` の失敗時に `return` するが、その場合 `thread_` メンバが**初期化されないまま**になる。デストラクタは無条件に `SDL_WaitThread(thread_)` を呼ぶため、ヘッドレス環境やディスプレイの無い環境で `--use-sdl` を指定すると未初期化ポインタで未定義動作 (クラッシュ) する。これを修正する。

## 現状

- `src/sdl_renderer/sdl_renderer.h` の `SDL_Thread* thread_;` はメンバ初期化リストに無い
- `src/sdl_renderer/sdl_renderer.cpp` のコンストラクタは初期化失敗時に `thread_` を設定せず return
- デストラクタ `~SDLRenderer()` は `SDL_WaitThread(thread_)` を無条件に呼ぶ

## 設計方針

- `thread_` を `nullptr` で初期化する
- デストラクタで `thread_ != nullptr` の場合のみ `SDL_WaitThread` を呼ぶ
- 初期化失敗時はエラーログを出力して処理を中断する (既存の挙動を維持)

## 完了条件

- SDL 初期化が失敗する環境で `--use-sdl` を指定してもクラッシュしない
- 通常環境では従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
