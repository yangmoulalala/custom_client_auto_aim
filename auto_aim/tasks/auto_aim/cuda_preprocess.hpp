#ifndef AUTO_AIM__CUDA_PREPROCESS_HPP
#define AUTO_AIM__CUDA_PREPROCESS_HPP

#include <cstddef>
#include <cstdint>
#include <cuda_runtime_api.h>

namespace auto_aim
{
cudaError_t launch_bgr_to_rgb_fp16(
  const std::uint8_t * bgr, void * fp16_nchw, std::size_t pixel_count, cudaStream_t stream);
}  // namespace auto_aim

#endif  // AUTO_AIM__CUDA_PREPROCESS_HPP
