#include "cuda_preprocess.hpp"

#include <cuda_fp16.h>

namespace auto_aim
{
namespace
{
__global__ void bgr_to_rgb_fp16_kernel(
  const std::uint8_t * bgr, half * fp16_nchw, std::size_t pixel_count)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= pixel_count) return;

  const auto bgr_index = index * 3;
  constexpr float scale = 1.0F / 255.0F;
  fp16_nchw[index] = __float2half_rn(static_cast<float>(bgr[bgr_index + 2]) * scale);
  fp16_nchw[pixel_count + index] =
    __float2half_rn(static_cast<float>(bgr[bgr_index + 1]) * scale);
  fp16_nchw[2 * pixel_count + index] =
    __float2half_rn(static_cast<float>(bgr[bgr_index]) * scale);
}
}  // namespace

cudaError_t launch_bgr_to_rgb_fp16(
  const std::uint8_t * bgr, void * fp16_nchw, std::size_t pixel_count, cudaStream_t stream)
{
  constexpr int block_size = 256;
  const auto grid_size = static_cast<unsigned int>((pixel_count + block_size - 1) / block_size);
  bgr_to_rgb_fp16_kernel<<<grid_size, block_size, 0, stream>>>(
    bgr, static_cast<half *>(fp16_nchw), pixel_count);
  return cudaGetLastError();
}
}  // namespace auto_aim
