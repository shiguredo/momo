# SDL 初期化失敗時に未初期化のスレッドハンドルを SDL_WaitThread に渡してクラッシュする

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-sdl-renderer-thread-init
- Polished: 2026-08-19
- Milestone: 2026.1.0

## 目的

`SDLRenderer` のコンストラクタは `SDL_Init` / `SDL_CreateWindow` / `SDL_CreateRenderer` (Apple のみ) の失敗時に `return` するが、その場合 `thread_` メンバが初期化されないままになる。デストラクタは無条件に `SDL_WaitThread(thread_)` を呼ぶため、ヘッドレス環境やディスプレイの無い環境で `--use-sdl` を指定すると未初期化ポインタで未定義動作 (クラッシュ) する。また `SDL_CreateThread` が `nullptr` を返した場合も同様にデストラクタでクラッシュする。これを修正する。

## 現状

- `src/sdl_renderer/sdl_renderer.h` の `SDLRenderer` クラスの `SDL_Thread* thread_;` はメンバ初期化子も初期化子リストもなく、デフォルト初期化されない。`src/sdl_renderer/sdl_renderer.cpp` の初期化子リストは `window_(nullptr)` / `renderer_(nullptr)` を持つが `thread_` が無い
- `src/sdl_renderer/sdl_renderer.cpp` の `SDLRenderer` コンストラクタは Apple 時に `SDL_Init` / `SDL_CreateWindow` / `SDL_CreateRenderer` のいずれかが失敗したときに `thread_` への代入 (`SDL_CreateThread` はコンストラクタ末尾) 前に `return` する。非 Apple では `SDL_CreateRenderer` 失敗は `RenderThread()` 内で起きるためコンストラクタ早期 `return` の原因にならない
- `SDL_CreateThread` の戻り値チェックがなく、失敗時に `thread_` が `nullptr` のままデストラクタで `SDL_WaitThread` される
- デストラクタ `SDLRenderer::~SDLRenderer()` は `thread_` の null チェックなしに `SDL_WaitThread(thread_)` を呼ぶ。`window_` / `renderer_` は既に `if (renderer_)` / `if (window_)` でガード済みだが `thread_` のみ未ガード
- 早期 `return` 後の `SDLRenderer` は `window_ == nullptr` / `renderer_ == nullptr` のゾンビ状態で `src/main.cpp` が `sdl_renderer.reset(new SDLRenderer(...))` 後に `RTCManager` へ渡す。`SDL_GetWindowFlags(window_)` 等で別経路のクラッシュが起きる可能性があるが、本 issue ではデストラクタのクラッシュ防止のみを責務とし、`main.cpp` 側のゾンビ検知はスコープ外とする

## 設計方針

- `src/sdl_renderer/sdl_renderer.h` で `SDL_Thread* thread_ = nullptr;` と NSDMI で初期化する (初期化子リストへの追加ではなくヘッダでの一本化で全経路を保証する)
- コンストラクタで `SDL_CreateThread` が `nullptr` を返した場合は英語で `RTC_LOG(LS_ERROR)` を出力して `return` する。デストラクタ `SDLRenderer::~SDLRenderer()` では `if (thread_ != nullptr)` の場合のみ `SDL_WaitThread(thread_)` を呼ぶ
- `window_` / `renderer_` は現行で `if (renderer_) SDL_DestroyRenderer` / `if (window_) SDL_DestroyWindow` とガード済みであることを確認し、変更せず維持する。`SDL_Quit()` は SDL3 では初期化失敗後でも安全なため維持する
- 初期化失敗時の挙動は既存通りコンストラクタ内で英語の `RTC_LOG(LS_ERROR)` を出力して `return` する。呼び出し元 `src/main.cpp` でのゾンビ検知やプロセス終了は本 issue のスコープ外とし、オブジェクトは生成されるが描画不能なまま継続する

## 完了条件

- ヘッドレス環境で `DISPLAY` を空にして `--use-sdl` を指定した際に、`SDL_Init` 失敗 / `SDL_CreateWindow` 失敗 / `SDL_CreateRenderer` 失敗 (macOS) / `SDL_CreateThread` 失敗のいずれでもデストラクタでクラッシュせず英語の `RTC_LOG(LS_ERROR)` が出力され、プロセスが `SIGTERM` なくデストラクタまで到達する
- 通常環境では従来通りウィンドウ表示とレンダリングが動作し、`SDL_CreateThread` 成功時に描画ループが回り `SIGTERM` で `SDL_WaitThread` が正常終了する

## 解決方法

未着手 (PR 作成後に追記する)

## pending にした理由

クラッシュはプロセス停止時のデストラクタで起きる。macOS 上で `SDL_VIDEODRIVER=invalid` と `--use-sdl` により初期化失敗までは再現できたが、終了時は `SDL Thread error:-1` のみで、未初期化 `thread_` によるクラッシュは再現しなかった。SDL3 の `SDL_WaitThread(NULL)` は no-op で status が `-1` になるため、クラッシュ前提の完了条件を今は満たせない。再現手段が固まるまで保留する。
