# MPU6500 Bare Metal Driver
![Bare Metal](https://img.shields.io/badge/Architecture-Bare_Metal-blue)
![STM32](https://img.shields.io/badge/Platform-STM32_HAL-green)
![Sensor](https://img.shields.io/badge/Sensor-MPU6500-orange)

## Project Overview
This project delivers a robust, bare-metal driver for the MPU6500 6-axis accelerometer and gyroscope. Designed for the STM32H5 microcontroller series, it provides a high-level API for sensor configuration and data acquisition while maintaining low-level control. The system includes a Python Matplotlib dashboard for real-time data logging and visualization, demonstrating a complete end-to-end embedded solution.

https://github.com/user-attachments/assets/35c72ef3-7cd2-4b26-8794-a8332a81c6cc

## Key Features
*   **Sensor Initialization & Verification**: Robust `WHO_AM_I` check to ensure hardware presence before configuration.
*   **Configurable Full-Scale Ranges**: Dynamic runtime configuration via direct register writing and bit-shifting:
    *   **Accelerometer**: ±2G, ±4G, ±8G, ±16G.
    *   **Gyroscope**: ±250, ±500, ±1000, ±2000 DPS.
*   **Real-Time Data Acquisition**: Efficient polling of all 6 axes with raw-to-float conversion using precise sensitivity scalers.
*   **Integrated Calibration**: Built-in routine to calculate and apply zero-rate offsets for the gyroscope, eliminating drift at startup.
*   **Memory Safety**: Strict adherence to `stdint.h` fixed-width types (`int16_t`, `uint8_t`) to ensure portable and predictable behavior.

## Hardware Requirements
*   **Microcontroller**: STM32H5 Series (STM32H503 - ARM Cortex-M33).
*   **Sensor**: MPU6500 (6-Axis IMU).
*   **Communication Interface**: I2C.
*   **Debug Output**: UART (utilized for Python visualization).

## Technical Implementation

### I2C Communication & Register Mapping
The driver handles register-level operations efficiently. All specific register addresses (e.g., `MPU6500_REG_ACCEL_CONFIG` `0x1C`) are macro-defined. Critical operations involve direct bit manipulation to ensure precise configuration without side effects.

### Register Configuration
Configuration functions utilize bitwise operations with Unsigned bits to safely modify specific bits in configuration registers without disturbing reserved or unrelated bits.

<details>
<summary><strong>View Actual Implementation: Register Bit-Shifting</strong></summary>

```c
// Example from mpu6500.c: Setting Accelerometer Range
HAL_StatusTypeDef MPU6500_SetAccelRange(I2C_HandleTypeDef *hi2c, Accel_Range range) {
  // Create a bit Mask (Inverted register value) to clear target bits
  // 0x18 = 0001 1000 -> ~0x18 = 0xE7 (1110 0111)
  const uint8_t inverted_range_mask = 0xE7;
  HAL_StatusTypeDef mpu_status;
  uint8_t Current_Register_Data = 0U;

  // 1. Read Current Register State
  mpu_status = HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG,
                       I2C_MEMADD_SIZE_8BIT, &Current_Register_Data, 1, 1000);
  
  // 2. Clear target bits
  uint8_t AND_ACCEL_Data = (Current_Register_Data & inverted_range_mask);

  if (mpu_status != HAL_OK) return -1;

  // 3. Apply new range with OR operation
  uint8_t Final_ACCEL_Data = (AND_ACCEL_Data | range);

  // 4. Write back to register
  mpu_status = HAL_I2C_Mem_Write(hi2c, MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &Final_ACCEL_Data, 1, 300);
  // ...
  return mpu_status;
}
```
</details>

### Data Processing
Raw sensor data (High and Low bytes) is combined into `int16_t` signed integers and then converted to floating-point units using a lookup table for sensitivity values.

<details>
<summary><strong>View Actual Implementation: Data Conversion</strong></summary>

```c
// Example from mpu6500.c: Reading Gyro Data
HAL_StatusTypeDef MPU6500_Read_Gyro_Data(I2C_HandleTypeDef *hi2c, MPU6500_Gyro_Data *Gyro_Data) {
  // ... (Reading raw_data from I2C) ...

  // Combine High and Low bytes
  Gyro_Data->Gyro_X = (raw_data[0] << 8) | raw_data[1];
  Gyro_Data->Gyro_Y = (raw_data[2] << 8) | raw_data[3];
  Gyro_Data->Gyro_Z = (raw_data[4] << 8) | raw_data[5];

  // Apply Sensitivity Scaling and Calibration Offset
  Gyro_Data->Gyro_X = (int16_t)((Gyro_Data->Gyro_X / gyro_norm_const) + MPUConfig.Gyro_Offset_Calibration[0]);
  Gyro_Data->Gyro_Y = (int16_t)((Gyro_Data->Gyro_Y / gyro_norm_const) + MPUConfig.Gyro_Offset_Calibration[1]);
  Gyro_Data->Gyro_Z = (int16_t)((Gyro_Data->Gyro_Z / gyro_norm_const) + MPUConfig.Gyro_Offset_Calibration[2]);

  return status;
}
```
</details>

## API Reference

### Initialization
```c
HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i);
```
Initializes the sensor, wakes it from sleep mode, and verifies the device ID.

### Configuration
```c
HAL_StatusTypeDef MPU6500_SetAccelRange(I2C_HandleTypeDef *hi2c, Accel_Range range);
HAL_StatusTypeDef MPU6500_SetRotationRange(I2C_HandleTypeDef *hi2c, Gyro_Range range);
```
Sets the full-scale range for the accelerometer and gyroscope.

### Data Reading
```c
HAL_StatusTypeDef MPU6500_Read_Accel_Data(I2C_HandleTypeDef *hi2c, MPU6500_Accel_Data *Accel_Data);
HAL_StatusTypeDef MPU6500_Read_Gyro_Data(I2C_HandleTypeDef *hi2c, MPU6500_Gyro_Data *Gyro_Data);
```
Reads the latest sensor data and populates the data structures with converted float values.

### Calibration
```c
HAL_StatusTypeDef MPU6500_Gyro_Calibration(I2C_HandleTypeDef *hi2c, int8_t return_offset[3]);
```
Performs a static calibration to determine zero-bias offsets. Configuration: 512 samples.

## Quick Start
Here is a minimal example of how to use the driver in your `main.c` loop:

```c
#include "mpu6500.h"

// 1. Initialize Objects
MPU6500_Gyro_Data Gyro_Data = {0};
MPU6500_Accel_Data Accel_Data = {0};
uint8_t who_am_i = 0;
int8_t gyro_offset[3] = {0};

// 2. Init and Calibrate
if (MPU6500_Init(&hi2c1, &who_am_i) == HAL_OK) {
    // Optional: Calibrate Gyro (Keep sensor still!)
    MPU6500_Gyro_Calibration(&hi2c1, gyro_offset);
}

// 3. Main Loop
while (1) {
    // Read Data
    MPU6500_Read_Gyro_Data(&hi2c1, &Gyro_Data);
    MPU6500_Read_Accel_Data(&hi2c1, &Accel_Data);

    // Process Data...
    
    HAL_Delay(10); // 100Hz Survey Rate
}
```

## Future Roadmap
- [ ] **DMA Integration**: Implement Non-blocking I2C transfers using Direct Memory Access to free up CPU cycles.
- [ ] **Interrupt Handling**: Utilize the MPU6500 `INT` pin (Data Ready) to trigger reads, replacing the current polling mechanism.

