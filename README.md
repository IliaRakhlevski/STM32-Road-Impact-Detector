# STM32 Road Impact Detector

## [Real Time College](https://rt-ed.co.il/) — a multi-disciplinary Real-Time O.S. and Embedded Software Solutions Center, providing consulting, development, integration, training, and support solutions.<br/>

## FreeRTOS Final Project

Course project developed as part of the FreeRTOS course.

The project implements a real-time road impact detection system based on the STM32F756ZG microcontroller. An ICM-20948 IMU is used to detect impact candidates, while a Quectel L80 GNSS receiver provides position, time, and a PPS signal for accurate event timestamping.

The system focuses on embedded firmware and does not attempt to classify detected events as potholes, speed bumps, or other specific road features.

---

## Hardware

- STM32 NUCLEO-F756ZG
- Pimoroni ICM-20948 IMU breakout board
- Quectel L80 GNSS module

![Project hardware](Docs/Images/nucleo-f756zg-board.jpeg)

---

## Technologies

- C
- STM32 HAL
- FreeRTOS
- CMSIS-RTOS V1
- I²C
- UART
- hardware timers and input capture
- GNSS NMEA and PPS

---

## Planned Functionality

- periodic IMU data acquisition
- road impact candidate detection
- GNSS NMEA reception and parsing
- PPS-based time synchronization
- high-resolution event timestamps
- FreeRTOS task synchronization and communication
- serial diagnostics through the ST-LINK Virtual COM Port

---

## Documentation

- [Architectural Design Decisions](Docs/Architectural-Design-Decisions.md)

The project is currently under development.


## Author

Ilia Rakhlevski
