#ifndef NVCODEC_H264_ENCODER_CUDA_H_
#define NVCODEC_H264_ENCODER_CUDA_H_

// CUDA と WebRTC のヘッダを混ぜてコンパイルすると酷いことになったので、
// CUDA の処理だけさせる単純な CUDA ファイルを用意する

// このヘッダーファイルは、外から呼ばれるので #include <cuda.h> をしてはいけない
// また、CUDA 側に WebRTC のヘッダを混ぜることができないので WebRTC のヘッダも include してはいけない

#include <NvEncoder/NvEncoder.h>

#include "cuda/cuda_context.h"

class NvCodecH264EncoderCudaImpl;

class NvCodecH264EncoderCuda {
 public:
  NvCodecH264EncoderCuda(std::shared_ptr<CudaContext> ctx);
  ~NvCodecH264EncoderCuda();

  void Copy(NvEncoder* nv_encoder, const void* ptr, int width, int height);
  // 念のため <memory> も include せずポインタを利用する
  NvEncoder* CreateNvEncoder(int width, int height, bool use_native);

 private:
  NvCodecH264EncoderCudaImpl* impl_;
};

#endif  // NVCODEC_H264_ENCODER_CUDA_H_
