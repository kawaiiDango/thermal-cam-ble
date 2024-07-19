/*!
 *  @file Adafruit_MLX90640.h
 *
 * 	I2C Driver for MLX90640 24x32 IR Thermal Camera
 *
 * 	This is a library for the Adafruit MLX90640 breakout:
 * 	https://www.adafruit.com/products/4407
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_MLX90640_H
#define _ADAFRUIT_MLX90640_H

#include <stdint.h>
#include "MLX90640_API.h"

#define MLX90640_I2CADDR_DEFAULT 0x33 ///< I2C address by default

#define MLX90640_DEVICEID1 0x2407 ///< I2C identification register

/** Mode to read pixel frames (two per image) */
typedef enum mlx90640_mode
{
  MLX90640_INTERLEAVED, ///< Read data from camera by interleaved lines
  MLX90640_CHESS,       ///< Read data from camera in alternating pixels
} mlx90640_mode_t;

/** Internal ADC resolution for pixel calculation */
typedef enum mlx90640_res
{
  MLX90640_ADC_16BIT,
  MLX90640_ADC_17BIT,
  MLX90640_ADC_18BIT,
  MLX90640_ADC_19BIT,
} mlx90640_resolution_t;

/** How many PAGES we will read per second (2 pages per frame) */
typedef enum mlx90640_refreshrate
{
  MLX90640_0_5_HZ,
  MLX90640_1_HZ,
  MLX90640_2_HZ,
  MLX90640_4_HZ,
  MLX90640_8_HZ,
  MLX90640_16_HZ,
  MLX90640_32_HZ,
  MLX90640_64_HZ,
} mlx90640_refreshrate_t;

#define OPENAIR_TA_SHIFT 8 ///< Default 8 degree offset from ambient air

int mlx_begin();

mlx90640_mode_t mlx_getMode(void);
void mlx_setMode(mlx90640_mode_t mode);
mlx90640_resolution_t mlx_getResolution(void);
void mlx_setResolution(mlx90640_resolution_t res);
mlx90640_refreshrate_t mlx_getRefreshRate(void);
void mlx_setRefreshRate(mlx90640_refreshrate_t res);

int mlx_getFrame(float *framebuf);

float mlx_getTa();

#endif
