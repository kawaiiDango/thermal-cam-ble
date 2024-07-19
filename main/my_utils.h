#ifndef H_MY_UTILS_
#define H_MY_UTILS_

#include <Adafruit_MLX90640.h>

// support only up to 8 hz
float refreshRateToHz(mlx90640_refreshrate_t rate)
{
  switch (rate)
  {
  case MLX90640_0_5_HZ:
    return 0.25f;
  case MLX90640_1_HZ:
    return 0.5f;
  case MLX90640_2_HZ:
    return 1.0f;
  case MLX90640_4_HZ:
    return 2.0f;
  case MLX90640_8_HZ:
    return 4.0f;
  case MLX90640_16_HZ:
    return 8.0f;
  default:
    return 2.0f;
  }
}

mlx90640_refreshrate_t hzToRefreshRate(float hz)
{
  if (hz <= 0.25f)
    return MLX90640_0_5_HZ;
  if (hz <= 0.5f)
    return MLX90640_1_HZ;
  if (hz <= 1.0f)
    return MLX90640_2_HZ;
  if (hz <= 2.0f)
    return MLX90640_4_HZ;
  if (hz <= 4.0f)
    return MLX90640_8_HZ;
  if (hz <= 8.0f)
    return MLX90640_16_HZ;
  return MLX90640_4_HZ;
}

#endif