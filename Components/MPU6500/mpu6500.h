#ifndef MPU6500_H
#define MPU6500_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

#define MPU6500_I2C_ADDR 0x68U
#define MPU6500_REG_WHO_AM_I 0x75U
#define MPU6500_REG_PWR_MGMT_1 0x6BU

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i);

#ifdef __cplusplus
}
#endif

#endif /* MPU6500_H */
