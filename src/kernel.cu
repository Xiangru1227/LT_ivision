#include <vector>
#include <iostream>
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "kernel.cuh"
#include <cuda_runtime.h>

#define BLOCK_X 16
#define BLOCK_Y 16

__device__ __forceinline__ uchar yuvToGreen(uchar Y, uchar U, uchar V) {
    int c = static_cast<int>(Y) - 16;
    int d = static_cast<int>(U) - 128;
    int e = static_cast<int>(V) - 128;

    int g = (298 * c - 100 * d - 208 * e + 128) >> 8;

    return static_cast<uchar>(min(max(g, 0), 255));
}

__global__ void yuv2green_kernel(const uchar* __restrict__ y,
                                 const uchar* __restrict__ u,
                                 const uchar* __restrict__ v,
                                 uchar* __restrict__ g,
                                 int width,
                                 int height,
                                 size_t pitch_u,
                                 size_t pitch_y,
                                 size_t pitch_g) {
    int tx = threadIdx.x;
    int ty = threadIdx.y;
    int x = blockIdx.x * blockDim.x + tx;
    int yIdx = blockIdx.y * blockDim.y + ty;

    if (x >= width || yIdx >= height) return;

    __shared__ uchar shared_u[BLOCK_Y / 2][BLOCK_X / 2];
    __shared__ uchar shared_v[BLOCK_Y / 2][BLOCK_X / 2];
    
    if ((ty % 2 == 0) && (tx % 2 == 0)) {
        int uv_x = blockIdx.x * (BLOCK_X / 2) + tx / 2;
        int uv_y = blockIdx.y * (BLOCK_Y / 2) + ty / 2;

        if (uv_x < width / 2 && uv_y < height / 2) {
            shared_u[ty / 2][tx / 2] = u[uv_y * pitch_u + uv_x];
            shared_v[ty / 2][tx / 2] = v[uv_y * pitch_u + uv_x];
        }
    }
    
    __syncthreads();
    
    uchar Yval = y[yIdx * pitch_y + x];
    
    int sx = tx / 2;
    int sy = ty / 2;
    uchar Uval = shared_u[sy][sx];
    uchar Vval = shared_v[sy][sx];
    
    g[yIdx * pitch_g + x] = yuvToGreen(Yval, Uval, Vval);
}

extern "C"
void launchYUV2Green_CUDA(const uchar* d_y, const uchar* d_u, const uchar* d_v,
                          uchar* d_green, int width, int height,
                          size_t pitch_u, size_t pitch_y, size_t pitch_g,
                          cudaStream_t stream)
{
    dim3 threads(BLOCK_X, BLOCK_Y);
    dim3 blocks((width + BLOCK_X - 1) / BLOCK_X,
                (height + BLOCK_Y - 1) / BLOCK_Y);

    yuv2green_kernel<<<blocks, threads, 0, stream>>>(
        d_y, d_u, d_v, d_green,
        width, height,
        pitch_u, pitch_y, pitch_g);
}
