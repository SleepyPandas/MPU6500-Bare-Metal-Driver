/**
 * @file mpu6500.c
 * @brief MPU6500 IMU Driver Source File
 *
 * This file contains the implementation of the functions declared in
 * mpu6500.h for the MPU6500 6-axis accelerometer and gyroscope driver.
 *
 * @author Anthony Hua ... Rather SleepyPandas
 */

#include "mpu6500.h"
#include "stm32_hal_legacy.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_i2c.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/*Global Variables --------------------------------------------------------*/

// Stores current device configuration for data processing
MPU6500_Config MPUConfig = {
    .Accel_Setting = MPU6500_Accel_2G,
    .Gyro_Setting = MPU6500_Gyro_250,
    .Gyro_Offset_Calibration = {0, 0, 0},
};

/* Sensitivity Lookup Tables -----------------------------------------------*/

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

/* Driver Implementation ---------------------------------------------------*/

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i) {
  HAL_StatusTypeDef mpu_status;
  const uint16_t dev_address = MPU6500_I2C_ADDR;
  const uint32_t timeout = 1000U;
  const uint8_t sleep_wake_mask = 0xBFU;

  uint8_t who_am_i_value = 0U;
  uint8_t pwr_mgmt_1_value = 0U;
  uint8_t wake = 0U;
  uint8_t verify = 0U;

  // Check device ID
  mpu_status =
      HAL_I2C_Mem_Read(hi2c, dev_address, MPU6500_REG_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &who_am_i_value, 1, timeout);

  if (who_am_i != NULL) {
    *who_am_i = who_am_i_value;
  } else {
    return HAL_ERROR;
  }

  // Wake device (clear sleep bit)
  mpu_status =
      HAL_I2C_Mem_Read(hi2c, dev_address, MPU6500_REG_PWR_MGMT_1,
                       I2C_MEMADD_SIZE_8BIT, &pwr_mgmt_1_value, 1, timeout);

  if (mpu_status != HAL_OK)
    return HAL_ERROR;

  wake = (uint8_t)(pwr_mgmt_1_value & sleep_wake_mask);

  mpu_status = HAL_I2C_Mem_Write(hi2c, dev_address, MPU6500_REG_PWR_MGMT_1,
                                 I2C_MEMADD_SIZE_8BIT, &wake, 1, timeout);

  if (mpu_status != HAL_OK)
    return HAL_ERROR;

  // Verify wake status
  mpu_status = HAL_I2C_Mem_Read(hi2c, dev_address, MPU6500_REG_PWR_MGMT_1,
                                I2C_MEMADD_SIZE_8BIT, &verify, 1, timeout);

  return mpu_status;
}

HAL_StatusTypeDef MPU6500_SetAccelRange(I2C_HandleTypeDef *hi2c,
                                        Accel_Range range) {
  const uint8_t inverted_range_mask = 0xE7; // ~0x18
  HAL_StatusTypeDef mpu_status;
  uint8_t Current_Register_Data = 0U;

  // Read current state
  mpu_status =
      HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG,
                       I2C_MEMADD_SIZE_8BIT, &Current_Register_Data, 1, 1000);

  // Clear bits
  uint8_t AND_ACCEL_Data = (Current_Register_Data & inverted_range_mask);

  if (mpu_status != HAL_OK) {
    return -1;
  }

  // Set new range
  uint8_t Final_ACCEL_Data = (AND_ACCEL_Data | range);

  mpu_status =
      HAL_I2C_Mem_Write(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &Final_ACCEL_Data, 1, 300);

  if (mpu_status != HAL_OK) {
    return -1;
  }

  // Update internal config
  int config = 0;
  switch (range) {
  case (MPU6500_ACC_SET_2G):
    config = MPU6500_Accel_2G;
    break;
  case (MPU6500_ACC_SET_4G):
    config = MPU6500_Accel_4G;
    break;
  case (MPU6500_ACC_SET_8G):
    config = MPU6500_Accel_8G;
    break;
  case (MPU6500_ACC_SET_16G):
    config = MPU6500_Accel_16G;
    break;
  default:
    config = MPU6500_Accel_2G;
  }
  MPUConfig.Accel_Setting = config;
  return mpu_status;
}

HAL_StatusTypeDef MPU6500_SetRotationRange(I2C_HandleTypeDef *hi2c,
                                           Gyro_Range range) {
  const uint8_t inverted_range_mask = 0xE7; // ~0x18
  HAL_StatusTypeDef mpu_status;
  uint8_t Current_Register_Data = 0U;

  // Read current state
  mpu_status =
      HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_GYRO_CONFIG,
                       I2C_MEMADD_SIZE_8BIT, &Current_Register_Data, 1, 1000);

  // Clear bits
  uint8_t AND_GYRO_Data = (Current_Register_Data & inverted_range_mask);

  if (mpu_status != HAL_OK) {
    return -1;
  }

  // Set new range
  uint8_t Final_GYRO_Data = (AND_GYRO_Data | range);

  mpu_status =
      HAL_I2C_Mem_Write(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_GYRO_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &Final_GYRO_Data, 1, 300);
  if (mpu_status != HAL_OK) {
    return -1;
  }

  // Update internal config
  int config = 0;
  switch (range) {
  case (MPU6500_Gyro_SET_250):
    config = MPU6500_Gyro_250;
    break;
  case (MPU6500_Gyro_SET_500):
    config = MPU6500_Gyro_500;
    break;
  case (MPU6500_Gyro_SET_1000):
    config = MPU6500_Gyro_1000;
    break;
  case (MPU6500_Gyro_SET_2000):
    config = MPU6500_Gyro_2000;
    break;
  default:
    config = MPU6500_Gyro_250;
  }
  MPUConfig.Gyro_Setting = config;
  return mpu_status;
}

HAL_StatusTypeDef MPU6500_Read_Gyro_Data(I2C_HandleTypeDef *hi2c,
                                         MPU6500_Gyro_Data *Gyro_Data) {
  uint8_t raw_data[6] = {0};
  HAL_StatusTypeDef status;
  float gyro_norm_const = get_gyro_sensitivity(MPUConfig.Gyro_Setting);

  status = HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_GYRO_MEASURE,
                            I2C_MEMADD_SIZE_8BIT, raw_data, 6, 100);

  if (status != HAL_OK)
    return status;

  Gyro_Data->Gyro_X = (raw_data[0] << 8) | raw_data[1];
  Gyro_Data->Gyro_Y = (raw_data[2] << 8) | raw_data[3];
  Gyro_Data->Gyro_Z = (raw_data[4] << 8) | raw_data[5];

  // Apply calibration offset
  Gyro_Data->Gyro_X = (int16_t)((Gyro_Data->Gyro_X / gyro_norm_const) +
                                MPUConfig.Gyro_Offset_Calibration[0]);
  Gyro_Data->Gyro_Y = (int16_t)((Gyro_Data->Gyro_Y / gyro_norm_const) +
                                MPUConfig.Gyro_Offset_Calibration[1]);
  Gyro_Data->Gyro_Z = (int16_t)((Gyro_Data->Gyro_Z / gyro_norm_const) +
                                MPUConfig.Gyro_Offset_Calibration[2]);

  return status;
}

HAL_StatusTypeDef MPU6500_Read_Accel_Data(I2C_HandleTypeDef *hi2c,
                                          MPU6500_Accel_Data *Accel_Data) {
  uint8_t raw_data[6] = {0};
  HAL_StatusTypeDef status;
  float accel_norm_const = get_accel_sensitivity(MPUConfig.Accel_Setting);

  // Read high and low bytes for X, Y, Z (6 bytes total)
  status = HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_MEASURE,
                            I2C_MEMADD_SIZE_8BIT, raw_data, 6, 100);

  if (status != HAL_OK)
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

HAL_StatusTypeDef MPU6500_Gyro_Calibration(I2C_HandleTypeDef *hi2c,
                                           int8_t return_offset[3]) {
  MPU6500_Gyro_Data gyro_data;
  int32_t accumulator_data[3] = {0};
  HAL_StatusTypeDef status;
  int8_t offset_data[3];

  // Collect samples to determine average offset (noise reduces with sqrt(N))
  // Using 512 samples
  for (int i = 0; i < 512; i++) {
    status = MPU6500_Read_Gyro_Data(hi2c, &gyro_data);
    if (status == HAL_OK) {
      accumulator_data[0] += gyro_data.Gyro_X;
      accumulator_data[1] += gyro_data.Gyro_Y;
      accumulator_data[2] += gyro_data.Gyro_Z;
    } else {
      return status;
    }
    HAL_Delay(1);
  }

  offset_data[0] =
      (int8_t)roundf(-1.0f * ((float)accumulator_data[0] / 512.0f));
  offset_data[1] =
      (int8_t)roundf(-1.0f * ((float)accumulator_data[1] / 512.0f));
  offset_data[2] =
      (int8_t)roundf(-1.0f * ((float)accumulator_data[2] / 512.0f));

  if (return_offset != NULL) {
    memcpy(return_offset, offset_data, sizeof(offset_data));
  }

  memcpy(MPUConfig.Gyro_Offset_Calibration, offset_data,
         sizeof(MPUConfig.Gyro_Offset_Calibration));

  return status;
}

