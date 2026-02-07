# MPU6500 Bare-Metal IMU Driver

![Language](https://img.shields.io/badge/Language-C-white) ![Architecture](https://img.shields.io/badge/Architecture-Bare_Metal-blue) ![Platform](https://img.shields.io/badge/Platform-STM32_ArmCortex_M33-42f5da) ![Sensor](https://img.shields.io/badge/Sensor-MPU6500-a442f5) ![Build](https://img.shields.io/badge/Build-CMake-e81526)

A from-scratch, register-level driver for the InvenSense MPU6500 6-axis IMU, running on an STM32H503RB (Arm Cortex-M33) with no external sensor libraries. Features a platform-agnostic architecture, non-blocking DMA transfers, automated gyroscope calibration, and a real-time Python visualization dashboard.

https://github.com/user-attachments/assets/083083cf-58ff-4d32-9ef0-c8d85db2905d

<!-- TODO: Photo of Physical setup maybe? -->

---

* [`Components/MPU6500/mpu6500.c`](Components/MPU6500/mpu6500.c)
* [`Components/MPU6500/mpu6500.h`](Components/MPU6500/mpu6500.h)
* [`Python Visualization Tool/MatPlotLib Dashboard.py`](Python%20Visualization%20Tool/MatPlotLib%20Dashboard.py)


| Skill | Implementation |
| :--- | :--- |
| **Platform-Agnostic Driver Design** | Function-pointer abstraction (`MPU6500_Config`) decouples sensor logic from any specific HAL -- portable to any MCU |
| **Non-Blocking DMA State Machine** | `IDLE -> READ_GYRO -> READ_ACCEL -> DATA_READY` loop frees the CPU between I2C transfers |
| **Safe Register Manipulation** | Read-modify-write with bitmasks preserves reserved bits during runtime configuration |
| **Automated Calibration** | 512-sample zero-rate offset averaging eliminates gyroscope drift at startup |
| **Dual API Surface** | Blocking calls for simplicity alongside split-phase DMA calls for real-time performance |
| **End-to-End Data Pipeline** | Firmware -> UART @ 115200 baud -> Python Matplotlib dashboard with real-time 3D STL visualization |



<!-- ## Architecture 
Probably lets put some kind of flowchart -->



---

## Project Structure

```
MPU6500-Bare-Metal-Driver/
├── Components/MPU6500/
│   ├── mpu6500.c              # Driver implementation (blocking + DMA APIs)
│   └── mpu6500.h              # Public API, register map, config struct
├── Core/Src/
│   └── main.c                 # DMA state machine, platform I2C wrappers, UART output
├── Python Visualization Tool/
│   ├── MatPlotLib Dashboard.py # Real-time dashboard with 3D STL rotation
│   └── requirements.txt
├── Drivers/                    # STM32 HAL & CMSIS (vendor-provided)
├── CMakeLists.txt
└── STM32H503xx_FLASH.ld       # Linker script
```

---

## Technical Deep Dive

### 1. HAL Abstraction Layer

The driver never calls STM32 HAL directly. Instead, platform I/O is injected through function pointers at initialization. This means the same driver source can run on any microcontroller -- only the three function pointers need to change.

```c
typedef struct {
    int8_t (*write)(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);
                    
    int8_t (*read)(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);
                   
    void   (*delay_ms)(uint32_t milliseconds);

    /* Internal state managed by the driver */
    Accel_Calculation  Accel_Setting;
    Gyro_Calculation   Gyro_Setting;
    int8_t             Gyro_Offset_Calibration[3];
} MPU6500_Config;
```

On the STM32 side, the platform wrappers map directly to DMA transfers:

```c
/* Non-blocking I2C register read via DMA */
int8_t stm32_read_DMA(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len) {
                      
    I2C1_RX_FLAG = 0;
    if (HAL_I2C_Mem_Read_DMA(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, len) == HAL_OK)
                             
        return 0;
    else
        return -1;
}

/* Wire the platform layer to the driver */
MPU6500_Config config = {
    .write    = stm32_write_DMA,
    .read     = stm32_read_DMA,
    .delay_ms = HAL_Delay,
};
```

### 2. Initialization & Wake-Up

The MPU6500 powers on in sleep mode. The driver reads `WHO_AM_I` to confirm the device is on the bus, then clears the `SLEEP` bit in `PWR_MGMT_1` using a bitmask, and verifies the write took effect.

```c
/* Confirm device identity */
config->read(MPU6500_I2C_ADDR, MPU6500_REG_WHO_AM_I, &who_am_i_value, 1);

/* Clear SLEEP bit (bit 6) with a safe mask */
wake = (uint8_t)(pwr_mgmt_1_value & MPU6500_SLEEP_WAKE_MASK);  // 0xBF
config->write(MPU6500_I2C_ADDR, MPU6500_REG_PWR_MGMT_1, &wake, 1);

/* Verify the device is awake */
config->read(MPU6500_I2C_ADDR, MPU6500_REG_PWR_MGMT_1, &verify, 1);
```

### 3. Safe Register Configuration (Read-Modify-Write)

Changing the accelerometer or gyroscope range touches bits `[4:3]` of the config register. The driver reads the current value first, clears only the target bits with an inverted mask, then ORs in the new setting. Reserved bits are never disturbed.

```c
const uint8_t range_mask = 0xE7;  // 1110 0111 — clears bits [4:3]

// 1. Read current register state
config->read(MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG, &reg_value, 1);

// 2. Clear target bits, preserve everything else
uint8_t new_value = (reg_value & range_mask) | range;

// 3. Write back
config->write(MPU6500_I2C_ADDR, MPU6500_REG_ACCEL_CONFIG, &new_value, 1);
```

### 4. Non-Blocking DMA State Machine

The main application loop uses a four-state machine to chain I2C DMA transfers without blocking the CPU. Each state kicks off a transfer and immediately yields -- the next iteration checks the DMA-complete flag before advancing.

```c
typedef enum { IDLE, READ_GYRO, READ_ACCEL, DATA_READY } MPU6500_State_t;

switch (MPU6500_current_state) {
case IDLE:
    MPU6500_Read_Gyro_DMA(&config, gyro_raw);
    MPU6500_current_state = READ_GYRO;
    break;

case READ_GYRO:
    if (I2C1_RX_FLAG) {
        I2C1_RX_FLAG = 0;
        MPU6500_Process_Gyro_DMA(gyro_raw, &Gyro_Data);
        MPU6500_Read_Accel_DMA(&config, accel_raw);
        MPU6500_current_state = READ_ACCEL;
    }
    break;

case READ_ACCEL:
    if (I2C1_RX_FLAG) {
        I2C1_RX_FLAG = 0;
        MPU6500_Process_Accel_DMA(accel_raw, &Accel_Data);
        MPU6500_current_state = DATA_READY;
    }
    break;

case DATA_READY:
    HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 100);
    MPU6500_current_state = IDLE;
    break;
}
/* CPU is free here for other non-blocking work */
```

### 5. Data Acquisition & Scaling

Raw sensor data arrives as big-endian high/low byte pairs over I2C. The driver combines them into signed 16-bit integers and normalizes with a sensitivity constant from the datasheet lookup table (e.g. 16384 LSB/g at +/-2G).

```c
// Combine high and low bytes into a signed 16-bit value
int16_t raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);

// Scale to engineering units using the datasheet sensitivity
Accel_Data->Accel_X = (float)(raw / accel_sensitivity);
```

### 6. Automated Gyroscope Calibration

At startup the device sits stationary while the driver collects 512 samples per axis. The average drift is computed and stored as a negative offset, which is then automatically applied to every subsequent reading.

```c
for (int i = 0; i < 512; i++) {
    MPU6500_Read_Gyro_DMA(config, gyro_raw);
    config->delay_ms(5);
    MPU6500_Process_Gyro_DMA(gyro_raw, &gyro_data);
    accumulator_data[0] += gyro_data.Gyro_X;
    // ... Y, Z
}

// Negative average cancels the zero-rate drift
offset[0] = (int8_t)roundf(-1.0f * ((float)accumulator_data[0] / 512.0f));
```

---

## Real-Time Visualization Dashboard

The Python dashboard connects over serial and renders live sensor data at 75 ms intervals:

- **Gyroscope plot** -- X, Y, Z angular velocity in degrees/sec
- **Accelerometer plot** -- X, Y, Z acceleration in g-force
- **3D STL viewer** -- loads a custom `.stl` model and rotates it in real time using integrated gyroscope data
- **Configurable window** -- adjustable X-axis sample window via an on-screen textbox with debounced input


**Source:** [`Python Visualization Tool/MatPlotLib Dashboard.py`](Python%20Visualization%20Tool/MatPlotLib%20Dashboard.py) | **Dependencies:** [`requirements.txt`](Python%20Visualization%20Tool/requirements.txt)

---

## Getting Started

### Hardware

| Component | Detail |
| :--- | :--- |
| **Microcontroller** | STM32H503RB Nucleo-64 (Arm Cortex-M33, 250 MHz) |
| **Sensor** | InvenSense MPU6500 (6-axis accelerometer + gyroscope) |
| **Communication** | I2C (Standard Mode) + UART @ 115200 baud |
| **Toolchain** | CMake, GCC ARM None EABI |

### Wiring

| MPU6500 Pin | STM32 Pin | Function |
| :---: | :---: | :---: |
| VCC | 3.3V | Power |
| GND | GND | Ground |
| SCL | PB8 (I2C1_SCL) | Clock Line |
| SDA | PB9 (I2C1_SDA) | Data Line |

### Build & Flash

```bash
# Configure
cmake --preset Debug

# Build
cmake --build build/Debug

# Flash with your preferred tool (e.g. STM32CubeProgrammer, OpenOCD)
```

### Usage Example

```c
/* Inject platform I/O into the driver */
MPU6500_Config config = {
    .write    = stm32_write_DMA,
    .read     = stm32_read_DMA,
    .delay_ms = HAL_Delay,
};

/* Initialize sensor (WHO_AM_I check + wake from sleep) */
MPU6500_Init(&config);

/* Calibrate gyroscope (512 samples, device must be stationary) */
int8_t offsets[3] = {0};
MPU6500_Gyro_Calibration(&config, offsets);

/* Main loop -- non-blocking reads */
while (1) {
    MPU6500_Read_Gyro_Data(&config, &Gyro_Data);
    MPU6500_Read_Accel_Data(&config, &Accel_Data);
}
```

### Python Dashboard

```bash
cd "Python Visualization Tool"
pip install -r requirements.txt
python "MatPlotLib Dashboard.py"
```

---

## Roadmap

- [ ] **Interrupt-Driven Reads** -- Use the MPU6500 `INT` pin (Data Ready) to trigger reads instead of continuous polling
- [ ] **SPI Support** -- Add an SPI transport option for higher-speed communication

---

## References

- [MPU-6500 Register Map (TDK InvenSense)](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf)
- [MPU-6500 Product Datasheet](https://datasheet.octopart.com/MPU-6500-InvenSense-datasheet-138896167.pdf)

---

**Driver Source:** [`mpu6500.c`](Components/MPU6500/mpu6500.c) | [`mpu6500.h`](Components/MPU6500/mpu6500.h) | **Application:** [`main.c`](Core/Src/main.c)
