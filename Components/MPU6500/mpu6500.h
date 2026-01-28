#ifndef MPU6500_H
#define MPU6500_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

#define MPU6500_I2C_ADDR (0x68U << 1) // Left Shifted 1 For I2C Address Spacing
#define MPU6500_REG_WHO_AM_I 0x75U
#define MPU6500_REG_PWR_MGMT_1 0x6BU
#define MPU6500_REG_ACCEL_CONFIG 0x1CU
#define MPU6500_REG_GYRO_CONFIG 0x1BU

typedef enum {
  MPU6500_ACC_FS_2G = 0x00,
  MPU6500_ACC_FS_4G = 0x08,  // 0000 1000
  MPU6500_ACC_FS_8G = 0x10,  // 0001 0000
  MPU6500_ACC_FS_16G = 0x18, // 0001 1000
} AccelRange;

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i);

HAL_StatusTypeDef MPU6500_SetAccelRange(I2C_HandleTypeDef *hi2c, AccelRange range);

HAL_StatusTypeDef MPU6500_SetRotationRange(I2C_HandleTypeDef *hi2c,
                                           uint8_t range);

#ifdef __cplusplus
}
#endif

#endif /* MPU6500_H */
