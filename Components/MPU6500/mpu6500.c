/**
 * @file mpu6500.c
 * @brief MPU6500 IMU driver implementation.
 *
 * Implements initialization, configuration, calibration, and data acquisition
 * for the MPU6500 6-axis accelerometer/gyroscope. Supports both blocking and
 * non-blocking (DMA) I2C through platform-supplied function pointers.
 *
 * @author Anthony Hua ... Rather SleepyPandas
 */

#include "mpu6500.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Sensitivity Lookup --------------------------------------------------------*/

/** @brief Returns the accelerometer LSB/g value for the given range. */
static float get_accel_sensitivity(Accel_Calculation sensitivity) {
  switch (sensitivity) {
  case MPU6500_Accel_2G:
    return 16384.0f;
  case MPU6500_Accel_4G:
    return 8192.0f;
  case MPU6500_Accel_8G:
    return 4096.0f;
  case MPU6500_Accel_16G:
    return 2048.0f;
  default:
    return 16384.0f;
  }
}

/** @brief Returns the gyroscope LSB/(deg/s) value for the given range. */
static float get_gyro_sensitivity(Gyro_Calculation sensitivity) {
  switch (sensitivity) {
  case MPU6500_Gyro_250:
    return 131.0f;
  case MPU6500_Gyro_500:
    return 65.5f;
  case MPU6500_Gyro_1000:
    return 32.8f;
  case MPU6500_Gyro_2000:
    return 16.4f;
  default:
    return 131.0f;
  }
}

/* Initialization & Configuration --------------------------------------------*/

int8_t MPU6500_Init(MPU6500_Config *config) {
  int8_t mpu_status;
  uint8_t who_am_i_value = 0U;
  uint8_t pwr_mgmt_1_value = 0U;
  uint8_t wake = 0U;
  uint8_t verify = 0U;

  /* Read WHO_AM_I register to confirm device is on the bus */
  mpu_status =
      config->read(MPU6500_I2C_ADDR, MPU6500_REG_WHO_AM_I, &who_am_i_value, 1);
  config->delay_ms(25); /* Allow I/O to complete (non-blocking compatibility) */

  if (mpu_status != 0)
    return -1;

  /* Read current power management state */
  mpu_status = config->read(MPU6500_I2C_ADDR, MPU6500_REG_PWR_MGMT_1,
                            &pwr_mgmt_1_value, 1);
  config->delay_ms(25);

  if (mpu_status != 0)
    return -1;

  /* Clear the SLEEP bit to wake the device */
  wake = (uint8_t)(pwr_mgmt_1_value & MPU6500_SLEEP_WAKE_MASK);

  mpu_status =
      config->write(MPU6500_I2C_ADDR, MPU6500_REG_PWR_MGMT_1, &wake, 1);
  config->delay_ms(25);

  if (mpu_status != 0)
    return -1;

  /* Verify the device is awake */
  mpu_status =
      config->read(MPU6500_I2C_ADDR, MPU6500_REG_PWR_MGMT_1, &verify, 1);
  config->delay_ms(25);

  return mpu_status;
}

int8_t MPU6500_SetAccelRange(MPU6500_Config *config, Accel_Range range) {
  const uint8_t range_mask = 0xE7; /* Inverted bits [4:3] */
  int8_t mpu_status;
  uint8_t reg_value = 0U;

  /* Read-modify-write: preserve other bits, set new range */
  mpu_status = config->read(MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG,
                            &reg_value, 1);
  config->delay_ms(10);

  if (mpu_status != 0)
    return -1;

  uint8_t new_value = (reg_value & range_mask) | range;

  mpu_status = config->write(MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG,
                             &new_value, 1);
  config->delay_ms(10);

  if (mpu_status != 0)
    return -1;

  /* Update internal sensitivity index */
  switch (range) {
  case MPU6500_ACC_SET_2G:
    config->Accel_Setting = MPU6500_Accel_2G;
    break;
  case MPU6500_ACC_SET_4G:
    config->Accel_Setting = MPU6500_Accel_4G;
    break;
  case MPU6500_ACC_SET_8G:
    config->Accel_Setting = MPU6500_Accel_8G;
    break;
  case MPU6500_ACC_SET_16G:
    config->Accel_Setting = MPU6500_Accel_16G;
    break;
  default:
    config->Accel_Setting = MPU6500_Accel_2G;
  }

  return mpu_status;
}

int8_t MPU6500_SetRotationRange(MPU6500_Config *config, Gyro_Range range) {
  const uint8_t range_mask = 0xE7; /* Inverted bits [4:3] */
  int8_t mpu_status;
  uint8_t reg_value = 0U;

  /* Read-modify-write: preserve other bits, set new range */
  mpu_status = config->read(MPU6500_I2C_ADDR, MPU6500_REG_GYRO_CONFIG,
                            &reg_value, 1);
  config->delay_ms(10);

  if (mpu_status != 0)
    return -1;

  uint8_t new_value = (reg_value & range_mask) | range;

  mpu_status = config->write(MPU6500_I2C_ADDR, MPU6500_REG_GYRO_CONFIG,
                             &new_value, 1);
  config->delay_ms(10);

  if (mpu_status != 0)
    return -1;

  /* Update internal sensitivity index */
  switch (range) {
  case MPU6500_Gyro_SET_250:
    config->Gyro_Setting = MPU6500_Gyro_250;
    break;
  case MPU6500_Gyro_SET_500:
    config->Gyro_Setting = MPU6500_Gyro_500;
    break;
  case MPU6500_Gyro_SET_1000:
    config->Gyro_Setting = MPU6500_Gyro_1000;
    break;
  case MPU6500_Gyro_SET_2000:
    config->Gyro_Setting = MPU6500_Gyro_2000;
    break;
  default:
    config->Gyro_Setting = MPU6500_Gyro_250;
  }

  return mpu_status;
}

/* Blocking Read API ---------------------------------------------------------*/

int8_t MPU6500_Read_Gyro_Data(MPU6500_Config *config,
                              MPU6500_Gyro_Data *Gyro_Data) {
  uint8_t raw_data[6] = {0};
  int8_t status;
  float gyro_norm_const = get_gyro_sensitivity(config->Gyro_Setting);

  status =
      config->read(MPU6500_I2C_ADDR, MPU6500_REG_GYRO_MEASURE, raw_data, 6);

  if (status != 0)
    return status;

  int16_t combined_data_raw[3] = {0};
  combined_data_raw[0] = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  combined_data_raw[1] = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  combined_data_raw[2] = (int16_t)((raw_data[4] << 8) | raw_data[5]);

  Gyro_Data->Gyro_X = (int16_t)((combined_data_raw[0] / gyro_norm_const) +
                                config->Gyro_Offset_Calibration[0]);
  Gyro_Data->Gyro_Y = (int16_t)((combined_data_raw[1] / gyro_norm_const) +
                                config->Gyro_Offset_Calibration[1]);
  Gyro_Data->Gyro_Z = (int16_t)((combined_data_raw[2] / gyro_norm_const) +
                                config->Gyro_Offset_Calibration[2]);

  return status;
}

int8_t MPU6500_Read_Accel_Data(MPU6500_Config *config,
                               MPU6500_Accel_Data *Accel_Data) {
  uint8_t raw_data[6] = {0};
  int8_t status;
  float accel_norm_const = get_accel_sensitivity(config->Accel_Setting);

  status =
      config->read(MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_MEASURE, raw_data, 6);

  if (status != 0)
    return status;

  int16_t combined_data_raw[3] = {0};
  combined_data_raw[0] = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  combined_data_raw[1] = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  combined_data_raw[2] = (int16_t)((raw_data[4] << 8) | raw_data[5]);

  Accel_Data->Accel_X = (float)(combined_data_raw[0] / accel_norm_const);
  Accel_Data->Accel_Y = (float)(combined_data_raw[1] / accel_norm_const);
  Accel_Data->Accel_Z = (float)(combined_data_raw[2] / accel_norm_const);

  return status;
}

/* Calibration ---------------------------------------------------------------*/

int8_t MPU6500_Gyro_Calibration(MPU6500_Config *config,
                                int8_t return_offset[3]) {
  MPU6500_Gyro_Data gyro_data;
  int32_t accumulator_data[3] = {0};
  int8_t status;
  int8_t offset_data[3];
  static uint8_t gyro_raw[6];

  /*
   * Collect 512 stationary samples using the split-phase DMA API.
   * The delay between Start and Process ensures data is available
   * regardless of whether the I/O is blocking or non-blocking.
   */
  for (int i = 0; i < 512; i++) {
    status = MPU6500_Read_Gyro_DMA(config, gyro_raw);
    config->delay_ms(1);
    MPU6500_Process_Gyro_DMA(config, gyro_raw, &gyro_data);

    if (status == 0) {
      accumulator_data[0] += gyro_data.Gyro_X;
      accumulator_data[1] += gyro_data.Gyro_Y;
      accumulator_data[2] += gyro_data.Gyro_Z;
    } else {
      return status;
    }
  }

  /* Compute negative average as the offset to cancel drift */
  offset_data[0] =
      (int8_t)roundf(-1.0f * ((float)accumulator_data[0] / 512.0f));
  offset_data[1] =
      (int8_t)roundf(-1.0f * ((float)accumulator_data[1] / 512.0f));
  offset_data[2] =
      (int8_t)roundf(-1.0f * ((float)accumulator_data[2] / 512.0f));

  if (return_offset != NULL) {
    memcpy(return_offset, offset_data, sizeof(offset_data));
  }

  memcpy(config->Gyro_Offset_Calibration, offset_data,
         sizeof(config->Gyro_Offset_Calibration));

  return status;
}

/* Non-blocking (DMA / Split-Phase) API --------------------------------------*/

int8_t MPU6500_Read_Gyro_DMA(MPU6500_Config *config, uint8_t raw_buf[6]) {
  return config->read(MPU6500_I2C_ADDR, MPU6500_REG_GYRO_MEASURE, raw_buf, 6);
}

int8_t MPU6500_Read_Accel_DMA(MPU6500_Config *config, uint8_t raw_buf[6]) {
  return config->read(MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_MEASURE, raw_buf, 6);
}

void MPU6500_Process_Gyro_DMA(MPU6500_Config *config, const uint8_t raw_buf[6],
                              MPU6500_Gyro_Data *data) {
  float gyro_norm_const = get_gyro_sensitivity(config->Gyro_Setting);

  int16_t combined_data_raw[3];
  combined_data_raw[0] = (int16_t)((raw_buf[0] << 8) | raw_buf[1]);
  combined_data_raw[1] = (int16_t)((raw_buf[2] << 8) | raw_buf[3]);
  combined_data_raw[2] = (int16_t)((raw_buf[4] << 8) | raw_buf[5]);

  data->Gyro_X = (int16_t)((combined_data_raw[0] / gyro_norm_const) +
                           config->Gyro_Offset_Calibration[0]);
  data->Gyro_Y = (int16_t)((combined_data_raw[1] / gyro_norm_const) +
                           config->Gyro_Offset_Calibration[1]);
  data->Gyro_Z = (int16_t)((combined_data_raw[2] / gyro_norm_const) +
                           config->Gyro_Offset_Calibration[2]);
}

void MPU6500_Process_Accel_DMA(MPU6500_Config *config, const uint8_t raw_buf[6],
                               MPU6500_Accel_Data *data) {
  float accel_norm_const = get_accel_sensitivity(config->Accel_Setting);

  int16_t combined_data_raw[3];
  combined_data_raw[0] = (int16_t)((raw_buf[0] << 8) | raw_buf[1]);
  combined_data_raw[1] = (int16_t)((raw_buf[2] << 8) | raw_buf[3]);
  combined_data_raw[2] = (int16_t)((raw_buf[4] << 8) | raw_buf[5]);

  data->Accel_X = (float)(combined_data_raw[0] / accel_norm_const);
  data->Accel_Y = (float)(combined_data_raw[1] / accel_norm_const);
  data->Accel_Z = (float)(combined_data_raw[2] / accel_norm_const);
}
