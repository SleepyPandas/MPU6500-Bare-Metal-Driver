#ifndef MPU6500_H
#define MPU6500_H

/**
 * @file mpu6500.h
 * @brief MPU6500 IMU Driver Header File
 *
 * This file contains the configuration structs, enums, and function prototypes
 * for the MPU6500 6-axis accelerometer and gyroscope driver.
 *
 * @author Anthony Hua ... Rather SleepyPandas
 */

#include <math.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

/** @brief MPU6500 I2C Address (Left Shifted) for Address Spacing + W/R bit */
#define MPU6500_I2C_ADDR (0x68U << 1)
#define Sleep_Wake_Mask 0xBFU
#define MPU6500_REG_WHO_AM_I 0x75U
#define MPU6500_REG_PWR_MGMT_1 0x6BU
#define MPU6500_REG_ACCEL_CONFIG 0x1CU
#define MPU6500_REG_GYRO_CONFIG 0x1BU

/** @brief Registers 59 to 64 – Accelerometer Measurements */
#define MPU6500_REG_ACCEL_MEASURE 0x3BU

/** @brief Registers 67 to 72 – Gyroscope Measurements */
#define MPU6500_REG_GYRO_MEASURE 0x43U //


typedef enum {
  MPU6500_ACC_SET_2G = 0x00,
  MPU6500_ACC_SET_4G = 0x08,  // 0000 1000
  MPU6500_ACC_SET_8G = 0x10,  // 0001 0000
  MPU6500_ACC_SET_16G = 0x18, // 0001 1000
} Accel_Range;


typedef enum {
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
  MPU6500_Accel_2G = 0,
  MPU6500_Accel_4G,
  MPU6500_Accel_8G,
  MPU6500_Accel_16G,
} Accel_Calculation;

// Internal enum for sensitivity scaling indexes
typedef enum {
  MPU6500_Gyro_250 = 0,
  MPU6500_Gyro_500,
  MPU6500_Gyro_1000,
  MPU6500_Gyro_2000,
} Gyro_Calculation;

// Driver Configuration State
typedef struct {
  int8_t (*write_DMA)(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);
  int8_t (*read_DMA)(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);
  int8_t (*delay)(uint32_t milliseconds);



  Accel_Calculation Accel_Setting;
  Gyro_Calculation Gyro_Setting;
  int8_t Gyro_Offset_Calibration[3];
} MPU6500_Config;

/**
 * @brief Initializes the MPU6500 sensor.
 *
 * Checks the device ID (WHO_AM_I) and wakes the device from sleep mode.
 *
 * @param hi2c Pointer to the I2C handle.
 * @param who_am_i Pointer to store the retrieved WHO_AM_I register value.
 * @return HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i);

/**
 * @brief Sets the full-scale range for the accelerometer.
 *
 * Updates the ACCEL_CONFIG register and the internal driver configuration.
 *
 * @param hi2c Pointer to the I2C handle.
 * @param range Desired accelerometer range (2G, 4G, 8G, 16G).
 * @return HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef MPU6500_SetAccelRange(I2C_HandleTypeDef *hi2c,
                                        Accel_Range range);

/**
 * @brief Sets the full-scale range for the gyroscope.
 *
 * Updates the GYRO_CONFIG register and the internal driver configuration.
 *
 * @param hi2c Pointer to the I2C handle.
 * @param range Desired gyroscope range (250, 500, 1000, 2000 dps).
 * @return HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef MPU6500_SetRotationRange(I2C_HandleTypeDef *hi2c,
                                           Gyro_Range range);

/**
 * @brief Reads the current angular rate data from the gyroscope.
 *
 * Reads raw data registers, applies sensitivity scaling, and subtracts
 * calibration offsets.
 *
 * @param hi2c Pointer to the I2C handle.
 * @param Gyro_Data Pointer to the struct where processed data will be stored.
 * @return HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef MPU6500_Read_Gyro_Data(I2C_HandleTypeDef *hi2c,
                                         MPU6500_Gyro_Data *Gyro_Data);

/**
 * @brief Reads the current acceleration data from the accelerometer.
 *
 * Reads raw data registers and applies sensitivity scaling.
 *
 * @param hi2c Pointer to the I2C handle.
 * @param Accel_Data Pointer to the struct where processed data will be stored.
 * @return HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef MPU6500_Read_Accel_Data(I2C_HandleTypeDef *hi2c,
                                          MPU6500_Accel_Data *Accel_Data);

/**
 * @brief Performs gyroscope calibration.
 *
 * Collects samples while the device is stationary to determine zero-rate
 * offsets. Updates the internal configuration with these offsets.
 *
 * @param hi2c Pointer to the I2C handle.
 * @param return_offset Pointer to an array of size 3 to store the calculated
 * offsets [X, Y, Z]. Can be NULL.
 * @return HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef MPU6500_Gyro_Calibration(I2C_HandleTypeDef *hi2c,
                                           int8_t return_offset[3]);





#ifdef __cplusplusso 
}
#endif

#endif /* MPU6500_H */
