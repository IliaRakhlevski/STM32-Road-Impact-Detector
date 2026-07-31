# STM32 Road Impact Detector

## [Real Time College](https://rt-ed.co.il/) — a multi-disciplinary Real-Time O.S. and Embedded Software Solutions Center, providing consulting, development, integration, training, and support solutions.<br/>

The project implements a real-time road impact detection system based on the STM32F756ZG microcontroller. An ICM-20948 IMU is used to detect impact candidates, while a Quectel LC86G-LA GNSS receiver provides position, time, and a PPS signal for accurate event timestamping.

The system focuses on embedded firmware and does not attempt to classify detected events as potholes, speed bumps, or other specific road features.

---

## Hardware

- STM32 NUCLEO-F756ZG
- Pimoroni ICM-20948 IMU breakout board
- Quectel LC86G-LA GNSS module on an HW-658 development board

| STM32 NUCLEO-F756ZG | Pimoroni ICM-20948 | Quectel LC86G-LA GNSS |
|:---:|:---:|:---:|
| <img src="Docs/Images/nucleo-f756zg-board.jpeg" alt="STM32 NUCLEO-F756ZG" width="240"> | <img src="Docs/Images/icm-20948-breakout-front.jpg" alt="Pimoroni ICM-20948" width="240"> | <img src="Docs/Images/quectel-l80-gnss-board-front.jpg" alt="Quectel LC86G-LA GNSS development board" width="240"> |

---

## Technologies

- C
- STM32 HAL
- FreeRTOS
- CMSIS-RTOS V1
- I²C
- UART
- Hardware Timers and Input Capture
- GNSS NMEA and PPS

---

## Current Implementation

- FreeRTOS Tasks for GNSS and IMU Acquisition
- GNSS NMEA Reception through UART with DMA
- GNGGA and GPGGA Position and UTC Parsing
- GNSS PPS Capture through a 32-Bit Hardware Timer
- Interrupt-Driven ICM-20948 Data Acquisition
- Acceleration Conversion to Units of g
- Common 1 MHz Timer Timestamps for GNSS PPS and IMU Interrupts
- PPS-Based UTC Synchronization of IMU Measurements
- Moving Acceleration Baseline and Impact Candidate Detection
- Association of Impact Candidates with UTC and GNSS Position
- Automatic I²C Recovery after IMU Communication Errors
- Thread-Safe Serial Diagnostics through the ST-LINK Virtual COM Port

## Planned Next Steps

- Controlled Road Tests and Detection Threshold Selection
- Reduction of Diagnostic Output for Normal Operation
- Event Queueing and Compact Host Reporting
- Stack, Heap, and Runtime Stability Measurements
- Watchdog and Extended Fault Handling

---

## Documentation

- [Architectural Design Decisions](Docs/Architectural-Design-Decisions.md)

The project is currently under development.


## Author

Ilia Rakhlevski
