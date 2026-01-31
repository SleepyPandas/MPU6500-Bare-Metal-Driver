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
#define MPU6500_REG_ACCEL_MEASURE                                              \
  0x3BU //  Registers 59 to 64 – Accelerometer Measurements
#define MPU6500_REG_GYRO_CONFIG 0x1BU
#define MPU6500_REG_GYRO_MEASURE                                               \
  0x43U // Registers 67 to 72 – Gyroscope Measurements

typedef enum {
  MPU6500_ACC_FS_2G = 0x00,
  MPU6500_ACC_FS_4G = 0x08,  // 0000 1000
  MPU6500_ACC_FS_8G = 0x10,  // 0001 0000
  MPU6500_ACC_FS_16G = 0x18, // 0001 1000
} AccelRange;

typedef struct {
  int16_t Gyro_X;
  int16_t Gyro_Y;
  int16_t Gyro_Z;
} MPU6500_Gyro_Data;

typedef struct {
  int16_t Accel_X;
  int16_t Accel_Y;
  int16_t Accel_Z;
} MPU6500_Accel_Data;

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i);

HAL_StatusTypeDef MPU6500_SetAccelRange(I2C_HandleTypeDef *hi2c,
                                        AccelRange range);

HAL_StatusTypeDef MPU6500_SetRotationRange(I2C_HandleTypeDef *hi2c,
                                           uint8_t range);

HAL_StatusTypeDef MPU6500_Read_Gyro_Data(I2C_HandleTypeDef *hi2c,
                                         MPU6500_Gyro_Data *Gyro_Data);

HAL_StatusTypeDef MPU6500_Read_Accel_Data(I2C_HandleTypeDef *hi2c,
                                          MPU6500_Accel_Data *Accel_Data);
#ifdef __cplusplus
}
#endif

#endif /* MPU6500_H */
