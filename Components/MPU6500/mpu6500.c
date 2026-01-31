

#include "mpu6500.h"
#include "stm32_hal_legacy.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_i2c.h"
#include <stdint.h>

/*Global Variables*/

// Purpose of this struct is to hold the current configuration of the MPU6500
// To be decoded when calculating data
MPU6500_Config MPUConfig = {
    .Accel_Setting = MPU6500_Accel_2G,
    .Gyro_Setting = MPU6500_Gyro_250,
};

/*LOOK UP TABLES FOR Sensor Sensitivities */

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

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i) {
  HAL_StatusTypeDef mpu_status;
  const uint16_t dev_address = MPU6500_I2C_ADDR;
  const uint32_t timeout = 1000U;
  const uint8_t sleep_wake_mask = 0xBFU;

  uint8_t who_am_i_value = 0U;
  uint8_t pwr_mgmt_1_value = 0U;
  uint8_t wake = 0U;
  uint8_t verify = 0U;

  /* Check if MPU-6500 is listening */
  mpu_status =
      HAL_I2C_Mem_Read(hi2c, dev_address, MPU6500_REG_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &who_am_i_value, 1, timeout);

  if (who_am_i != NULL) {
    *who_am_i = who_am_i_value;
  }

  /* Read, modify, and write PWR_MGMT_1 to wake device (clear sleep bit) */
  mpu_status =
      HAL_I2C_Mem_Read(hi2c, dev_address, MPU6500_REG_PWR_MGMT_1,
                       I2C_MEMADD_SIZE_8BIT, &pwr_mgmt_1_value, 1, timeout);

  wake = (uint8_t)(pwr_mgmt_1_value & sleep_wake_mask);

  mpu_status = HAL_I2C_Mem_Write(hi2c, dev_address, MPU6500_REG_PWR_MGMT_1,
                                 I2C_MEMADD_SIZE_8BIT, &wake, 1, timeout);

  /* Verify wake status */
  mpu_status = HAL_I2C_Mem_Read(hi2c, dev_address, MPU6500_REG_PWR_MGMT_1,
                                I2C_MEMADD_SIZE_8BIT, &verify, 1, timeout);

  return mpu_status;
}

/**
 * @param  scale: The desired MPU6500_ACC (2G,4G,8G,16G).
 * @note default is 0x00, 2G
 */

HAL_StatusTypeDef MPU6500_SetAccelRange(I2C_HandleTypeDef *hi2c,
                                        Accel_Range range) {
  // So we have to Create a bit Bask (Inverted register value) to clear the
  // bits we want to change 0x18 = 0001 1000

  const uint8_t inverted_range_mask = ~(uint8_t)range;
  HAL_StatusTypeDef mpu_status;
  // temporary variable to hold register data
  uint8_t Current_Register_Data = 0U;

  // Read, And, range OR , Write

  mpu_status =
      HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG,
                       I2C_MEMADD_SIZE_8BIT, &Current_Register_Data, 1, 1000);
  // CLEAR DATA BITS
  uint8_t AND_ACCEL_Data = (Current_Register_Data & inverted_range_mask);

  if (mpu_status != HAL_OK) {
    return -1;
  }

  uint8_t Final_ACCEL_Data = (AND_ACCEL_Data | range);

  mpu_status =
      HAL_I2C_Mem_Write(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &Final_ACCEL_Data, 1, 300);

  if (mpu_status != HAL_OK) {
    return -1;
  }

  // Want to change the MPUConfig struct here to reflect new sensitivity
  MPUConfig.Accel_Setting = range;
  return mpu_status;
}

/**
 * @param  scale: The desired MPU6500_Rotation (250,500,1000,2000).
 * @note default is 0x00, 250 deg/s
 */
HAL_StatusTypeDef MPU6500_SetRotationRange(I2C_HandleTypeDef *hi2c,
                                           Gyro_Range range) {
  const uint8_t inverted_range_mask = ~(uint8_t)range;
  HAL_StatusTypeDef mpu_status;
  // temporary variable to hold register data
  uint8_t Current_Register_Data = 0U;

  // Read, And, range OR , Write

  mpu_status =
      HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG,
                       I2C_MEMADD_SIZE_8BIT, &Current_Register_Data, 1, 1000);
  // CLEAR DATA BITS
  uint8_t AND_GYRO_Data = (Current_Register_Data & inverted_range_mask);

  if (mpu_status != HAL_OK) {
    return -1;
  }

  uint8_t Final_GYRO_Data = (AND_GYRO_Data | range);

  mpu_status =
      HAL_I2C_Mem_Write(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_GYRO_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &Final_GYRO_Data, 1, 300);
  if (mpu_status != HAL_OK) {
    return -1;
  }

  // Want to change the MPUConfig struct here to reflect new sensitivity
  MPUConfig.Gyro_Setting = range;
  return mpu_status;
}

HAL_StatusTypeDef MPU6500_Read_Gyro_Data(I2C_HandleTypeDef *hi2c,
                                         MPU6500_Gyro_Data *Gyro_Data) {
  uint8_t raw_data[6] = {0};
  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_GYRO_MEASURE,
                            I2C_MEMADD_SIZE_8BIT, raw_data, 6, 100);

  if (status != HAL_OK)
    return status;

  // Convert Raw Data into Gyro deg/s Acceleration
  // Formula is Raw Gyro / Sensitivity Default it to 131.0 for now

  Gyro_Data->Gyro_X = (raw_data[0] << 8) | raw_data[1];
  Gyro_Data->Gyro_Y = (raw_data[2] << 8) | raw_data[3];
  Gyro_Data->Gyro_Z = (raw_data[4] << 8) | raw_data[5];

  Gyro_Data->Gyro_X = (int16_t)(Gyro_Data->Gyro_X / 131.0);
  Gyro_Data->Gyro_Y = (int16_t)(Gyro_Data->Gyro_Y / 131.0);
  Gyro_Data->Gyro_Z = (int16_t)(Gyro_Data->Gyro_Z / 131.0);

  return status;
}

HAL_StatusTypeDef MPU6500_Read_Accel_Data(I2C_HandleTypeDef *hi2c,
                                          MPU6500_Accel_Data *Accel_Data) {
  uint8_t raw_data[6] = {0};
  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_MEASURE,
                            I2C_MEMADD_SIZE_8BIT, raw_data, 6, 100);

  if (status != HAL_OK)
    return status;

  // Convert Raw Data into G's Acceleration
  // Formula is Raw Accel / Sensitivity Default it to 2G for now

  int16_t combined_data_raw[3] = {0};

  combined_data_raw[0] = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  combined_data_raw[1] = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  combined_data_raw[2] = (int16_t)((raw_data[4] << 8) | raw_data[5]);

  Accel_Data->Accel_X = (float)(combined_data_raw[0] / 16384.0f);
  Accel_Data->Accel_Y = (float)(combined_data_raw[1] / 16384.0f);
  Accel_Data->Accel_Z = (float)(combined_data_raw[2] / 16384.0f);

  return status;
}
