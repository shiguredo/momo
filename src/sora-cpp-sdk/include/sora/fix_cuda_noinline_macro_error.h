/*
## 背景

WebRTC を M121 に更新した際に以下のビルド・エラーが発生した

```
error: use of undeclared identifier 'noinline'; did you mean 'inline'
```

WebRTC に含まれる libcxx のバージョンが更新されたことが原因だと思われる

## 対応

同様の問題を解消したと思われる LLVM の [PR](https://github.com/llvm/llvm-project-release-prs/pull/698) を調査したところ、 PR で追加されたファイルは存在するにも関わらず、問題が継続して発生していることがわかった
(libcxx に bits/basic_string.h が含まれておらず、 cuda_wrappers 以下のファイルがインクルードされていないようだった)

上記 PR を参考に、ファイルを直接修正したところエラーが解消したため、このヘッダー・ファイルをエラーが発生する箇所でインクルードすることにした
オリジナルのパッチには push_macro や pop_macro が含まれているが、省いても問題が無かったため省略している

*/

#undef __noinline__