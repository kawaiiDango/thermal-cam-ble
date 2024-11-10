#include <Adafruit_MLX90640.h>
#include <MLX90640_I2C_Driver.h>
#include <config.h>
#include <stdio.h>

paramsMLX90640 _params;
float ta = -999.0;
uint16_t serialNumber[3]; ///< Unique serial number read from device

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_addr
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
int mlx_begin()
{
  MLX90640_I2CInit();

  // wire->setClock(400000); // Speed it up, lots to read :)
  MLX90640_I2CRead(0, MLX90640_DEVICEID1, 3, serialNumber);

  uint16_t eeMLX90640[MLX90640_EEPROM_DUMP_NUM];
  if (MLX90640_DumpEE(0, eeMLX90640) != 0)
  {
    return -1;
  }
#ifdef MLX90640_DEBUG
  for (int i = 0; i < 832; i++)
  {
    printf("0x%x, ", eeMLX90640[i]);
  }
  printf("\n");
#endif

  MLX90640_ExtractParameters(eeMLX90640, &_params);
  // whew!
  return 0;
}

/*!
 *    @brief Get the frame-read mode
 *    @return Chess or interleaved mode
 */
mlx90640_mode_t mlx_getMode(void)
{
  return (mlx90640_mode_t)MLX90640_GetCurMode(0);
}

/*!
 *    @brief Set the frame-read mode
 *    @param mode Chess or interleaved mode
 */
void mlx_setMode(mlx90640_mode_t mode)
{
  if (mode == MLX90640_CHESS)
  {
    MLX90640_SetChessMode(0);
  }
  else
  {
    MLX90640_SetInterleavedMode(0);
  }
}

/*!
 *    @brief  Get resolution for temperature precision
 *    @returns The desired resolution (bits)
 */
mlx90640_resolution_t mlx_getResolution(void)
{
  return (mlx90640_resolution_t)MLX90640_GetCurResolution(0);
}

/*!
 *    @brief  Set resolution for temperature precision
 *    @param res The desired resolution (bits)
 */
void mlx_setResolution(mlx90640_resolution_t res)
{
  MLX90640_SetResolution(0, (int)res);
}

/*!
 *    @brief  Get max refresh rate
 *    @returns How many pages per second to read (2 pages per frame)
 */
mlx90640_refreshrate_t mlx_getRefreshRate(void)
{
  return (mlx90640_refreshrate_t)MLX90640_GetRefreshRate(0);
}

/*!
 *    @brief  Set max refresh rate - too fast and we can't read the
 *    the pages in time, start low and then increment while speeding
 *    up I2C!
 *    @param rate How many pages per second to read (2 pages per frame)
 */
void mlx_setRefreshRate(mlx90640_refreshrate_t rate)
{
  MLX90640_SetRefreshRate(0, (int)rate);
}

/*!
 *    @brief  Read 2 pages, calculate temperatures and place into framebuf
 *    @param  framebuf 24*32 floating point memory buffer
 *    @return 0 on success
 */
int mlx_getFrame(float *framebuf, float emissivity)
{
  float tr = 23.15;
  uint16_t mlx90640Frame[834];
  int status;

  for (uint8_t page = 0; page < 2; page++)
  {
    status = MLX90640_GetFrameData(0, mlx90640Frame);

#ifdef MLX90640_DEBUG
    printf("Page%d = [", page);
    for (int i = 0; i < 834; i++)
    {
      printf("0x%x, ", mlx90640Frame[i]);
    }
    printf("]\n");
#endif

    if (status < 0)
    {
      return status;
    }

    ta = MLX90640_GetTa(mlx90640Frame, &_params); // Store ambient temp locally
    tr = ta - OPENAIR_TA_SHIFT;                   // For a MLX90640 in the open air the shift is
                                                  // -8 degC.
#ifdef MLX90640_DEBUG
    printf("Tr = %.8f\n", tr);
#endif
    MLX90640_CalculateTo(mlx90640Frame, &_params, emissivity, tr, framebuf);
  }
  return 0;
}

/*!
 *    @brief  Return ambient temperature of the TO39 package.
 *    @param  newFrame If true, will also capture a new data frame. If false,
 * return the value from the last data frame read.
 *    @return Ambient temperature as a float in degrees Celsius.
 */
float mlx_getTa()
{
  return ta;
}
