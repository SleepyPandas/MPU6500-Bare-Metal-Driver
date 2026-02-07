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

#ifdef __cplusplus
extern "C" {
#endif

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
  int8_t (*write)(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data,
                  uint16_t len);
  int8_t (*read)(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data,
                 uint16_t len);
  void (*delay_ms)(uint32_t milliseconds);

  Accel_Calculation Accel_Setting;
  Gyro_Calculation Gyro_Setting;
  int8_t Gyro_Offset_Calibration[3];
} MPU6500_Config;

/**
 * @brief Initializes the MPU6500 sensor.
 *
 * Checks the device ID (WHO_AM_I) and wakes the device from sleep mode.
 *
 *
 * @param who_am_i Pointer to store the retrieved WHO_AM_I register value.
 * @return 0 for success, -1 for failure
 */
int8_t MPU6500_Init(MPU6500_Config *config);

/**
 * @brief Sets the full-scale range for the accelerometer.
 *
 * Updates the ACCEL_CONFIG register and the internal driver configuration.
 *
 *
 * @param range Desired accelerometer range (2G, 4G, 8G, 16G).
 * @return 0 for success, -1 for failure
 */
int8_t MPU6500_SetAccelRange(MPU6500_Config *config, Accel_Range range);

/**
 * @brief Sets the full-scale range for the gyroscope.
 *
 * Updates the GYRO_CONFIG register and the internal driver configuration.
 *
 *
 * @param range Desired gyroscope range (250, 500, 1000, 2000 dps).
 * @return 0 for success, -1 for failure
 */
int8_t MPU6500_SetRotationRange(MPU6500_Config *config, Gyro_Range range);

/**
 * @brief Reads the current angular rate data from the gyroscope.
 *
 * Reads raw data registers, applies sensitivity scaling, and subtracts
 * calibration offsets.
 *
 *
 * @param Gyro_Data Pointer to the struct where processed data will be stored.
 * @return 0 for success, -1 for failure
 */
int8_t MPU6500_Read_Gyro_Data(MPU6500_Config *config,
                              MPU6500_Gyro_Data *Gyro_Data);

/**
 * @brief Reads the current acceleration data from the accelerometer.
 *
 * Reads raw data registers and applies sensitivity scaling.
 *
 *
 * @param Accel_Data Pointer to the struct where processed data will be stored.
 * @return 0 for success, -1 for failure
 */
int8_t MPU6500_Read_Accel_Data(MPU6500_Config *config,
                               MPU6500_Accel_Data *Accel_Data);

/**
 * @brief Performs gyroscope calibration.
 *
 * Collects samples while the device is stationary to determine zero-rate
 * offsets. Updates the internal configuration with these offsets.
 *
 *
 * @param return_offset Pointer to an array of size 3 to store the calculated
 * offsets [X, Y, Z]. Can be NULL.
 * @return 0 for success, -1 for failure
 */
int8_t MPU6500_Gyro_Calibration(MPU6500_Config *config,
                                int8_t return_offset[3]);

/** @brief Below are Functions Assuming the user is Using DMA or Non blocking */
int8_t MPU6500_Read_Gyro_DMA(MPU6500_Config *config, uint8_t raw_buf[6]);

int8_t MPU6500_Read_Accel_DMA(MPU6500_Config *config, uint8_t raw_buf[6]);

void MPU6500_Process_Gyro_DMA(const uint8_t raw_buf[6],
                              MPU6500_Gyro_Data *data);

void MPU6500_Process_Accel_DMA(const uint8_t raw_buf[6],
                               MPU6500_Accel_Data *data);

#ifdef __cplusplus
}
#endif

#endif /* MPU6500_H */
