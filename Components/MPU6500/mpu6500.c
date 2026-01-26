

#include "mpu6500.h"

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i) {
  HAL_StatusTypeDef mpu_status;
  const uint16_t dev_address = (uint16_t)(MPU6500_I2C_ADDR << 1);
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

  mpu_status =
      HAL_I2C_Mem_Write(hi2c, dev_address, MPU6500_REG_PWR_MGMT_1,
                        I2C_MEMADD_SIZE_8BIT, &wake, 1, timeout);

  /* Verify wake status */
  mpu_status =
      HAL_I2C_Mem_Read(hi2c, dev_address, MPU6500_REG_PWR_MGMT_1,
                       I2C_MEMADD_SIZE_8BIT, &verify, 1, timeout);

  return mpu_status;
}
