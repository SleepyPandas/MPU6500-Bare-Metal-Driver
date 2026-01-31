#ifndef MPU6500_H
#define MPU6500_H

#include <math.h>
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
  // Full Scale Select
  MPU6500_ACC_SET_2G = 0x00,
  MPU6500_ACC_SET_4G = 0x08,  // 0000 1000
  MPU6500_ACC_SET_8G = 0x10,  // 0001 0000
  MPU6500_ACC_SET_16G = 0x18, // 0001 1000
} Accel_Range;

typedef enum {
  // Full Scale Select
  MPU6500_Gyro_SET_250 = 0x00,
  MPU6500_Gyro_SET_500 = 0x08,  // 0000 1000
  MPU6500_Gyro_SET_1000 = 0x10, // 0001 0000
  MPU6500_Gyro_SET_2000 = 0x18, // 0001 1000
} Gyro_Range;

typedef struct {
  int16_t Gyro_X;
  int16_t Gyro_Y;
  int16_t Gyro_Z;
} MPU6500_Gyro_Data;

typedef struct {
  float Accel_X;
  float Accel_Y;
  float Accel_Z;
} MPU6500_Accel_Data;

typedef enum {
  // Full-Scale Range,Sensitivity Scale Factor LSB/g
  MPU6500_Accel_2G = 0,
  MPU6500_Accel_4G,
  MPU6500_Accel_8G,
  MPU6500_Accel_16G,
} Accel_Calculation;

typedef enum {
  MPU6500_Gyro_250 = 0,
  MPU6500_Gyro_500,
  MPU6500_Gyro_1000,
  MPU6500_Gyro_2000,
} Gyro_Calculation;

typedef struct {
  // remember Null Terminator
  char Accel_Config[3];
  char Gyro_Config[5];
  float Accel_Calculation;
  float Gyro_Acceleration;
} MPU6500_Config;


HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i);

HAL_StatusTypeDef MPU6500_SetAccelRange(I2C_HandleTypeDef *hi2c,
                                        Accel_Range range);

HAL_StatusTypeDef MPU6500_SetRotationRange(I2C_HandleTypeDef *hi2c,
                                           Gyro_Range range);

HAL_StatusTypeDef MPU6500_Read_Gyro_Data(I2C_HandleTypeDef *hi2c,
                                         MPU6500_Gyro_Data *Gyro_Data);

HAL_StatusTypeDef MPU6500_Read_Accel_Data(I2C_HandleTypeDef *hi2c,
                                          MPU6500_Accel_Data *Accel_Data);
#ifdef __cplusplus
}
#endif

#endif /* MPU6500_H */
