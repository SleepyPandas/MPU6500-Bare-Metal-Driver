#ifndef MPU6500_H
#define MPU6500_H

/**
 * @file mpu6500.h
 * @brief Platform-agnostic MPU6500 6-axis IMU driver.
 *
 * Provides both a blocking API (for polling/blocking I2C) and a split-phase
 * non-blocking API (for DMA or interrupt-driven I2C). The driver is decoupled
 * from any specific HAL through user-supplied function pointers in
 * MPU6500_Config.
 *
 * @author Anthony Hua ... Rather SleepyPandas
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* I2C Address ---------------------------------------------------------------*/

/** @brief MPU6500 I2C address (0x68), left-shifted for HAL compatibility. */
#define MPU6500_I2C_ADDR (0x68U << 1)

/* Register Map --------------------------------------------------------------*/


/** @brief Bitmask to clear the SLEEP bit in PWR_MGMT_1 (bit 6). */
#define MPU6500_SLEEP_WAKE_MASK 0xBFU


#define MPU6500_REG_WHO_AM_I 0x75U
#define MPU6500_REG_PWR_MGMT_1 0x6BU
#define MPU6500_REG_ACCEL_CONFIG 0x1CU
#define MPU6500_REG_GYRO_CONFIG 0x1BU

/** @brief Registers 59 to 64 – Accelerometer Measurements */
#define MPU6500_REG_ACCEL_MEASURE 0x3BU

/** @brief Registers 67 to 72 – Gyroscope Measurements */
#define MPU6500_REG_GYRO_MEASURE 0x43U //



/* Enums — Range Selection ---------------------------------------------------*/

/** @brief Full-scale accelerometer range register values. */
typedef enum {
  MPU6500_ACC_SET_2G = 0x00,
  MPU6500_ACC_SET_4G = 0x08,
  MPU6500_ACC_SET_8G = 0x10,
  MPU6500_ACC_SET_16G = 0x18,
} Accel_Range;

/** @brief Full-scale gyroscope range register values. */
typedef enum {
  MPU6500_Gyro_SET_250 = 0x00,
  MPU6500_Gyro_SET_500 = 0x08,
  MPU6500_Gyro_SET_1000 = 0x10,
  MPU6500_Gyro_SET_2000 = 0x18,
} Gyro_Range;

/* Enums — Internal Sensitivity Indexes --------------------------------------*/

/** @brief Internal index for accelerometer sensitivity lookup. */
typedef enum {
  MPU6500_Accel_2G = 0,
  MPU6500_Accel_4G,
  MPU6500_Accel_8G,
  MPU6500_Accel_16G,
} Accel_Calculation;

/** @brief Internal index for gyroscope sensitivity lookup. */
typedef enum {
  MPU6500_Gyro_250 = 0,
  MPU6500_Gyro_500,
  MPU6500_Gyro_1000,
  MPU6500_Gyro_2000,
} Gyro_Calculation;

/* Data Structs --------------------------------------------------------------*/

/** @brief Processed gyroscope output in degrees per second (dps). */
typedef struct {
  int16_t Gyro_X;
  int16_t Gyro_Y;
  int16_t Gyro_Z;
} MPU6500_Gyro_Data;

/** @brief Processed accelerometer output in g-force (g). */
typedef struct {
  float Accel_X;
  float Accel_Y;
  float Accel_Z;
} MPU6500_Accel_Data;

/* Driver Configuration ------------------------------------------------------*/

/**
 * @brief Driver configuration and platform abstraction.
 *
 * Users must populate the three function pointers before calling any driver
 * function. The remaining fields are managed internally.
 */
typedef struct {
  /**
   * @brief Write to a device register over I2C.
   * @param dev_addr  Left-shifted 7-bit I2C address.
   * @param reg_addr  Register address to write to.
   * @param p_data    Pointer to data buffer.
   * @param len       Number of bytes to write.
   * @return 0 on success, -1 on failure.
   */
  int8_t (*write)(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data,
                  uint16_t len);

  /**
   * @brief Read from a device register over I2C.
   * @param dev_addr  Left-shifted 7-bit I2C address.
   * @param reg_addr  Register address to read from.
   * @param p_data    Pointer to buffer that will receive the data.
   * @param len       Number of bytes to read.
   * @return 0 on success, -1 on failure.
   */
  int8_t (*read)(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data,
                 uint16_t len);

  /**
   * @brief Millisecond delay function.
   * @param milliseconds  Duration to wait.
   */
  void (*delay_ms)(uint32_t milliseconds);

  Accel_Calculation Accel_Setting;       /**< Current accel range (internal). */
  Gyro_Calculation Gyro_Setting;         /**< Current gyro range (internal). */
  int8_t Gyro_Offset_Calibration[3];     /**< Gyro zero-rate offsets [X,Y,Z]. */
} MPU6500_Config;

/* Blocking API — assumes config->read()/write() return with data ready ------*/

/**
 * @brief Initialize the MPU6500 sensor.
 *
 * Reads WHO_AM_I, clears the SLEEP bit, and verifies the device is awake.
 * Uses delay_ms() between I2C operations for non-blocking compatibility.
 *
 * @param config  Pointer to a fully populated MPU6500_Config.
 * @return 0 on success, -1 on failure.
 */
int8_t MPU6500_Init(MPU6500_Config *config);

/**
 * @brief Set the accelerometer full-scale range.
 * @param config  Driver configuration.
 * @param range   Desired range (2G, 4G, 8G, or 16G).
 * @return 0 on success, -1 on failure.
 */
int8_t MPU6500_SetAccelRange(MPU6500_Config *config, Accel_Range range);

/**
 * @brief Set the gyroscope full-scale range.
 * @param config  Driver configuration.
 * @param range   Desired range (250, 500, 1000, or 2000 dps).
 * @return 0 on success, -1 on failure.
 */
int8_t MPU6500_SetRotationRange(MPU6500_Config *config, Gyro_Range range);

/**
 * @brief Read and process gyroscope data in a single blocking call.
 *
 * Reads 6 raw bytes, applies sensitivity scaling and calibration offsets.
 *
 * @note Assumes config->read() is blocking (data is in the buffer on return).
 *
 * @param config     Driver configuration.
 * @param Gyro_Data  Output struct for processed gyro values (dps).
 * @return 0 on success, -1 on failure.
 */
int8_t MPU6500_Read_Gyro_Data(MPU6500_Config *config,
                              MPU6500_Gyro_Data *Gyro_Data);

/**
 * @brief Read and process accelerometer data in a single blocking call.
 *
 * Reads 6 raw bytes and applies sensitivity scaling.
 *
 * @note Assumes config->read() is blocking (data is in the buffer on return).
 *
 * @param config      Driver configuration.
 * @param Accel_Data  Output struct for processed accel values (g).
 * @return 0 on success, -1 on failure.
 */
int8_t MPU6500_Read_Accel_Data(MPU6500_Config *config,
                               MPU6500_Accel_Data *Accel_Data);

/**
 * @brief Calibrate gyroscope zero-rate offsets.
 *
 * Collects 512 samples while the device is stationary and averages them to
 * compute per-axis offsets. Stores the result in the internal driver config.
 * Uses the split-phase DMA functions with delay_ms() between each sample.
 *
 * @param config         Driver configuration.
 * @param return_offset  Optional output array [X,Y,Z] for the offsets. Can be NULL.
 * @return 0 on success, -1 on failure.
 */
int8_t MPU6500_Gyro_Calibration(MPU6500_Config *config,
                                int8_t return_offset[3]);

/* Non-blocking (DMA / split-phase) API --------------------------------------*/

/**
 * @brief Start a gyroscope I2C read into a caller-owned buffer.
 *
 * Initiates a 6-byte read from the gyro data registers. For non-blocking
 * implementations, this returns immediately; the caller must wait for I/O
 * completion before calling MPU6500_Process_Gyro_DMA().
 *
 * @note Caller must ensure raw_buf remains valid until the transfer completes.
 *
 * @param config   Driver configuration.
 * @param raw_buf  Caller-owned 6-byte buffer (must not be a local/stack variable
 *                 if using non-blocking I/O).
 * @return 0 on success, -1 on failure.
 */
int8_t MPU6500_Read_Gyro_DMA(MPU6500_Config *config, uint8_t raw_buf[6]);

/**
 * @brief Start an accelerometer I2C read into a caller-owned buffer.
 *
 * Initiates a 6-byte read from the accel data registers. For non-blocking
 * implementations, this returns immediately; the caller must wait for I/O
 * completion before calling MPU6500_Process_Accel_DMA().
 *
 * @note Caller must ensure raw_buf remains valid until the transfer completes.
 *
 * @param config   Driver configuration.
 * @param raw_buf  Caller-owned 6-byte buffer (must not be a local/stack variable
 *                 if using non-blocking I/O).
 * @return 0 on success, -1 on failure.
 */
int8_t MPU6500_Read_Accel_DMA(MPU6500_Config *config, uint8_t raw_buf[6]);

/**
 * @brief Process a completed gyroscope raw buffer into scaled output.
 *
 * Pure computation — no I/O. Combines raw bytes, applies sensitivity scaling
 * and calibration offsets. Call only after the I2C transfer has completed.
 *
 * @param raw_buf  6-byte buffer containing completed gyro register data.
 * @param data     Output struct for processed gyro values (dps).
 */
void MPU6500_Process_Gyro_DMA(MPU6500_Config *config, const uint8_t raw_buf[6],
                              MPU6500_Gyro_Data *data);

/**
 * @brief Process a completed accelerometer raw buffer into scaled output.
 *
 * Pure computation — no I/O. Combines raw bytes and applies sensitivity
 * scaling. Call only after the I2C transfer has completed.
 *
 * @param raw_buf  6-byte buffer containing completed accel register data.
 * @param data     Output struct for processed accel values (g).
 */
void MPU6500_Process_Accel_DMA(MPU6500_Config *config, const uint8_t raw_buf[6],
                               MPU6500_Accel_Data *data);

#ifdef __cplusplus
}
#endif

#endif /* MPU6500_H */
