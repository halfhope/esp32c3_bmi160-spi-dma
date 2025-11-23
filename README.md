# ESP32 Sketch for BMI160 (FreeRTOS + SPI + DMA)

## Overview
This sketch implements full initialization and operation with the **BMI160** inertial sensor (accelerometer + gyroscope) on the **ESP32-C3** platform using **FreeRTOS**, **SPI with DMA**, and **UART for data output**.  
The code includes:

- SPI initialization with manual CS control via GPIO
- Implementation of SPI read/write functions for the `bmi160.h` library
- Soft reset of the sensor, CHIP ID verification, and error register checks
- Configuration of accelerometer (±4g, 100 Hz) and gyroscope (±1000 dps, 100 Hz)
- **FOC (Fast Offset Calibration)** for precise accelerometer calibration
- Manual bias calibration for accelerometer and gyroscope
- Two FreeRTOS tasks:
  - `sensor_task` - streaming raw data via UART
  - `perf_task` - printing task runtime stats every 5 seconds

## Notes

I recommend using a different sensor. If you plan to use the IMU for spatial orientation, the BMI160 gradually accumulates drift over time, and the data becomes unreliable.  

I suggest switching to another IMU that has an additional data source - a magnetometer. For example, the ICM-20948 (note: it requires a 3.3V–5V logic level converter).

## Pin Configuration
| Function      | GPIO |
|---------------|------|
| SPI MISO      | 5    |
| SPI MOSI      | 6    |
| SPI SCLK      | 4    |
| SPI CS (manual) | 7    |

## Requirements
- ESP-IDF (with FreeRTOS support)
- `bmi160.h` library (Bosch Sensortec)
- BMI160 connected via SPI
