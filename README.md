# MPU6500 Bare Metal Driver
![Language](https://img.shields.io/badge/Language-C-white)
![Architecture](https://img.shields.io/badge/Architecture-Bare_Metal-blue)
![Platform](https://img.shields.io/badge/Platform-STM32_HAL-42f5da)
![Sensor](https://img.shields.io/badge/Sensor-MPU6500-a442f5)

[View Driver Components Code](https://github.com/SleepyPandas/MPU6500-Bare-Metal-Driver/tree/main/Components/MPU6500) | [View Python Visualization Tool Code](https://github.com/SleepyPandas/MPU6500-Bare-Metal-Driver/tree/main/Python%20Visualization%20Tool)

## Project Overview
This project implements a modular bare-metal driver for the MPU6500 6-axis accelerometer and gyroscope on the STM32H503RB Nucleo-64 development board. The driver is split into a dedicated component (`Components/MPU6500`) and is called from `main.c` to read the sensor and stream data over UART to a Python Matplotlib dashboard for visualization.

This code does not use an existing library for the MPU6500 instead this driver manually handles register interaction to ensure maximum control and efficiency. It demonstrates precise configuration using safe read-modify-write patterns, automated startup calibration, and raw data sensitivity scaling without relying on heavy external dependencies.

https://github.com/user-attachments/assets/35c72ef3-7cd2-4b26-8794-a8332a81c6cc

## Key Features
 **Bare Metal Implementation**: Direct register manipulation using STM32 HAL for I2C communication. <br>
**Configurable Full-Scale Ranges**: Dynamic runtime configuration via direct register writing and bit-shifting:
  *   **Accelerometer**: ±2G, ±4G, ±8G, ±16G. <br>
  *   **Gyroscope**: ±250, ±500, ±1000, ±2000 DPS. <br>
  
**On-Startup Calibration**: Integrated routine that samples 512 datapoints at startup to calculate and apply Gyroscope offsets. <br> 
**Modular Design**: Sensor logic is decoupled from the main application loop, utilizing a clean configuration struct MPU6500_Config  

## Hardware & Tech Stack
**Microcontroller:** STM32H503 (Arm Cortex-M33) <br>
**Sensor:** InvenSense MPU6500 (6-Axis IMU) <br>
**Communication:** I2C (Standard Mode) <br>
**Output:** UART Serial Console and Python Matplotlib Dashboard <br>
**Toolchain:** CMake, GCC ARM None EABI 

## Technical Deep Dive

### 1. Driver Initialization & Wake-Up
The MPU6500 starts in sleep mode. The driver explicitly handles the wake-up sequence by modifying the `PWR_MGMT_1` register, verifying device readiness before any data operations.

```c
/* Check if MPU-6500 is listening */
HAL_I2C_Mem_Read(hi2c, dev_address, MPU6500_REG_WHO_AM_I, ...);

/* Read, modify, and write PWR_MGMT_1 to wake device (clear sleep bit) */
wake = (uint8_t)(pwr_mgmt_1_value & sleep_wake_mask);
HAL_I2C_Mem_Write(hi2c, dev_address, MPU6500_REG_PWR_MGMT_1, ..., &wake, ...);
```

### 2. Safe Configuration (Read-Modify-Write)
To prevent overwriting reserved bits, the driver uses a strict masking pattern when changing sensor ranges (e.g., ±2G to ±16G). This ensures robust operation during runtime configuration changes.

```c
// Example: Setting Accelerometer Range
const uint8_t inverted_range_mask = 0xE7; // 1110 0111

// 1. Read Current State
HAL_I2C_Mem_Read(hi2c, ..., &Current_Register_Data, ...);

// 2. Clear target bits
uint8_t AND_ACCEL_Data = (Current_Register_Data & inverted_range_mask);

// 3. Apply new range
uint8_t Final_ACCEL_Data = (AND_ACCEL_Data | range);
```

### 3. Data Acquisition & Scaling
Raw sensor data is read as high/low byte pairs via I2C, combined into 16-bit integers, and normalized using a sensitivity lookup table to produce human-readable units (G or deg/s).

```c
// Combine High and Low bytes
Gyro_Data->Gyro_X = (raw_data[0] << 8) | raw_data[1];

// Apply Sensitivity Scaling
Gyro_Data->Gyro_X = (int16_t)((Gyro_Data->Gyro_X / gyro_norm_const) 
                    + MPUConfig.Gyro_Offset_Calibration[0]);
```

### 4. Automated Startup Calibration
A dedicated routine captures 512 samples at startup to calculate the Zero-Rate Error (drift). This average offset is effectively subtracted from all subsequent readings for higher accuracy.

```c
// Sample 512 datapoints for noise reduction
for (int i = 0; i < 512; i++) {
    MPU6500_Read_Gyro_Data(hi2c, &gyro_data);
    accumulator_data[0] += gyro_data.Gyro_X;
    // ...
}

// Calculate average offset
offset_data[0] = (int8_t)roundf(-1.0f * ((float)accumulator_data[0] / 512.0f));
```


## Quick Start

### Wiring

| MPU6500 Pin | STM32 Pin | Function |
| --- | --- | --- |
| VCC | 3.3V | Power |
| GND | GND | Ground |
| SCL | PB8 (I2C1_SCL) | Clock Line |
| SDA | PB9 (I2C1_SDA) | Data Line |

### Usage Example

```c
/* Initialize I2C and Sensor */
MPU6500_Init(&hi2c1, &who_am_i);

/* Run Calibration */
int8_t gyro_config[3] = {0};
MPU6500_Gyro_Calibration(&hi2c1, gyro_config);

/* Main Loop */
while (1) {
    MPU6500_Read_Gyro_Data(&hi2c1, &Gyro_Data);
    MPU6500_Read_Accel_Data(&hi2c1, &Accel_Data);
}
    
```

## Future Roadmap
- [ ] **DMA Integration**: Implement Non-blocking I2C transfers using Direct Memory Access to free up CPU cycles.
- [ ] **Interrupt Handling**: Utilize the MPU6500 `INT` pin (Data Ready) to trigger reads, replacing the current polling mechanism.

## Datasheets
* [MPU-6500 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf)
* [MPU-6500 Datasheet](https://datasheet.octopart.com/MPU-6500-InvenSense-datasheet-138896167.pdf)

