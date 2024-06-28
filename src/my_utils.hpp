#ifndef H_MY_UTILS_
#define H_MY_UTILS_

#include <Arduino.h>
#include <Adafruit_MLX90640.h>

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  const float run = in_max - in_min;
  if (run == 0)
  {
    log_e("map(): Invalid input range, min == max");
    return -1; // AVR returns -1, SAM returns 0
  }
  const float rise = out_max - out_min;
  const float delta = x - in_min;
  return (delta * rise) / run + out_min;
}

// support only up to 8 fps
float refreshRateToFps(mlx90640_refreshrate_t rate)
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

mlx90640_refreshrate_t fpsToRefreshRate(float fps)
{
  if (fps <= 0.25f)
    return MLX90640_0_5_HZ;
  if (fps <= 0.5f)
    return MLX90640_1_HZ;
  if (fps <= 1.0f)
    return MLX90640_2_HZ;
  if (fps <= 2.0f)
    return MLX90640_4_HZ;
  if (fps <= 4.0f)
    return MLX90640_8_HZ;
  if (fps <= 8.0f)
    return MLX90640_16_HZ;
  return MLX90640_4_HZ;
}

#endif