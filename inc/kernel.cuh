#pragma once

#include <cuda_runtime.h>
#include <opencv2/core.hpp>

#ifdef __cplusplus
extern "C" {
#endif

void launchYUV2Green_CUDA(const uchar* d_y, const uchar* d_u, const uchar* d_v,
                          uchar* d_green, int width, int height,
                          size_t pitch_u, size_t pitch_y, size_t pitch_g,
                          cudaStream_t stream);

#ifdef __cplusplus
}
#endif
