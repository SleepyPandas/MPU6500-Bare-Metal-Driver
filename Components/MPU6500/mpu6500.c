

#include "mpu6500.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_i2c.h"
#include <stdint.h>

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
                                        AccelRange range) {
  // So we have to Create a bit Bask
  const uint8_t acc_clear_mask = ~(uint8_t)0x18U;
  HAL_StatusTypeDef mpu_status;
  uint8_t MPU_GYRO_Data = 0U;

  // Read, And, range OR , Write

  mpu_status = HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_GYRO_CONFIG,
                                I2C_MEMADD_SIZE_8BIT, &MPU_GYRO_Data, 1, 1000);
  uint8_t AND_GYRO_Data = (MPU_GYRO_Data & acc_clear_mask);

  // experimenting way to error out
  if (mpu_status == HAL_ERROR || mpu_status == HAL_TIMEOUT) {
    return -1;
  }

  uint8_t Final_GYRO_Data = (AND_GYRO_Data | range);

  mpu_status =
      HAL_I2C_Mem_Write(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_GYRO_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &Final_GYRO_Data, 1, 300);

  return mpu_status;
}

/**
 * @param  scale: The desired MPU6500_Rotation (250,500,1000,2000).
 * @note default is 0x00, 250 deg/s
 */
HAL_StatusTypeDef MPU6500_SetRotationRange(I2C_HandleTypeDef *hi2c,
                                           uint8_t range) {
  // To be implemented
}


HAL_StatusTypeDef MPU6500_Read_Gyro_Data(I2C_HandleTypeDef *hi2c, MPU6500_Gyro_Data *Gyro_Data) {
  uint8_t raw_data[6] = {0};
  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_GYRO_MEASURE, I2C_MEMADD_SIZE_8BIT, raw_data, 6, 100);

  if (status == HAL_ERROR) return -1;

  Gyro_Data->Gyro_X = (raw_data[0] << 8) | raw_data[1];
  Gyro_Data->Gyro_Y = (raw_data[2] << 8) | raw_data[3];
  Gyro_Data->Gyro_Z = (raw_data[4] << 8) | raw_data[5];

}

HAL_StatusTypeDef MPU6500_Read_Accel_Data(I2C_HandleTypeDef *hi2c, MPU6500_Accel_Data *Accel_Data) {
  uint8_t raw_data[6] = {0};
}
