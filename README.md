# STM32 Road Impact Detector

## [Real Time College](https://rt-ed.co.il/) — a multi-disciplinary Real-Time O.S. and Embedded Software Solutions Center, providing consulting, development, integration, training, and support solutions.<br/>

The project implements a real-time road impact detection system based on the STM32F756ZG microcontroller. An ICM-20948 IMU is used to detect impact candidates, while a Quectel L80 GNSS receiver provides position, time, and a PPS signal for accurate event timestamping.

The system focuses on embedded firmware and does not attempt to classify detected events as potholes, speed bumps, or other specific road features.

---

## Hardware

- STM32 NUCLEO-F756ZG
- Pimoroni ICM-20948 IMU breakout board
- Quectel L80 GNSS module

| STM32 NUCLEO-F756ZG | Pimoroni ICM-20948 | Quectel L80 GNSS |
|:---:|:---:|:---:|
| <img src="Docs/Images/nucleo-f756zg-board.jpeg" alt="STM32 NUCLEO-F756ZG" width="240"> | <img src="Docs/Images/icm-20948-breakout-front.jpg" alt="Pimoroni ICM-20948" width="240"> | <img src="Docs/Images/quectel-l80-gnss-board-front.jpg" alt="Quectel L80 GNSS" width="240"> |

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

## Planned Functionality

- Periodic IMU Data Acquisition
- Road Impact Candidate Detection
- GNSS NMEA Reception and Parsing
- PPS-Based Time Synchronization
- High-Resolution Event Timestamps
- FreeRTOS Task Synchronization and Communication
- Serial Diagnostics through the ST-LINK Virtual COM Port

---

## Documentation

- [Architectural Design Decisions](Docs/Architectural-Design-Decisions.md)

The project is currently under development.


## Author

Ilia Rakhlevski
