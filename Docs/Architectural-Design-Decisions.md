# Architectural Design Decisions

This document records the main architectural and design decisions of the STM32 Road Impact Detector project.

Its purpose is to explain **why** a solution was selected. Peripheral settings and implementation details belong in the source code and dedicated technical documentation.


## Project Scope

The system detects road impact events and associates them with GNSS position and time data.

It does not attempt to classify an event as a pothole, speed bump, road joint, or another specific road feature. Reliable semantic classification would require additional sensing and processing, such as a camera and image analysis, which are outside the embedded focus of this project.

The project therefore concentrates on:

- Real-Time Sensor Acquisition
- Impact Candidate Detection
- Accurate Event Timestamps
- GNSS Position Acquisition
- FreeRTOS Synchronization and Inter-Task Communication
- Embedded Diagnostics and Fault Handling


## Target Hardware

The target platform is the NUCLEO-F756ZG development board with an STM32F756ZGT6 microcontroller.

![NUCLEO-F756ZG development board](Images/nucleo-f756zg-board.jpeg)

The board provides sufficient processing and memory resources, multiple communication peripherals, 32-bit timers, an integrated ST-LINK debugger, and a Virtual COM Port. This makes it suitable for demonstrating a realistic RTOS-based embedded system without requiring a custom PCB.

The project uses real sensors rather than simulated data:

- Pimoroni ICM-20948 breakout board for motion acquisition
- Quectel L80 GNSS module on an HW-658 development board

![Pimoroni ICM-20948 breakout board](Images/icm-20948-breakout-front.jpg)

![Quectel L80 GNSS development board](Images/quectel-l80-gnss-board-front.jpg)

Additional views:

- [ICM-20948 breakout board — back](Images/icm-20948-breakout-back.jpg)
- [Quectel L80 GNSS development board — back](Images/quectel-l80-gnss-board-back.jpg)

Real hardware exposes practical embedded-development problems that simulated input does not reproduce: device initialization, electrical interfaces, communication failures, asynchronous data arrival, sensor noise, and timing uncertainty.

Relevant documentation:

- [NUCLEO-F756ZG product page](https://www.st.com/en/evaluation-tools/nucleo-f756zg.html)
- [UM1974 — STM32 Nucleo-144 boards user manual](https://www.st.com/resource/en/user_manual/dm00244518-stm32-nucleo-144-boards-mb1137-stmicroelectronics.pdf)
- [STM32F756ZG product page](https://www.st.com/en/microcontrollers-microprocessors/stm32f756zg.html)
- [STM32F756xx datasheet](https://www.st.com/resource/en/datasheet/stm32f756zg.pdf)
- [RM0385 — STM32F75xxx/STM32F74xxx reference manual](https://www.st.com/resource/en/reference_manual/rm0385-stm32f75xxx-and-stm32f74xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [ICM-20948 product page and datasheet](https://www.invensense.tdk.com/en-us/products/9-axis/icm-20948/)
- [Pimoroni ICM-20948 breakout board](https://shop.pimoroni.com/products/icm20948)
- [Quectel L80 Hardware Design](https://quectel.com/content/uploads/2021/03/Quectel_L80_Hardware_Design_V1.5.pdf)
- [Quectel LG77L(C), Lx0, Lx6 and LC86L Series GNSS Protocol Specification V2.2](https://www.quectel.com/content/uploads/2024/04/Quectel_LG77LCLx0Lx6LC86L_Series_GNSS_Protocol_Specification_V2.2.pdf)


## FreeRTOS Interface

FreeRTOS is configured through CMSIS-RTOS V1.

This interface is consistent with the course environment and has already been used successfully in previous projects. Using a familiar and proven interface reduces middleware risk while development focuses on sensor acquisition, timing, and synchronization.


## FreeRTOS Memory Management

The project uses the `heap_4` memory manager.

It supports allocation and deallocation and merges adjacent free blocks, reducing external fragmentation compared with `heap_2`. The initial heap size is a provisional budget and will be reviewed after the final RTOS object set is defined.

Task stacks will be sized individually and validated during testing. The generated default task is intentionally assigned only the minimum stack because it currently has no application responsibility.

`configUSE_NEWLIB_REENTRANT` is enabled so each task receives its own newlib reentrancy state. This improves safety when newlib functions are called from multiple tasks, but increases per-task RAM usage and must be included in memory analysis.

Reference:

- [FreeRTOS heap memory management](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/09-Memory-management/01-Memory-management)


## System Clock Source

The system uses the 8 MHz external clock supplied by the ST-LINK MCO output. STM32CubeMX therefore configures the clock input as **HSE Bypass**.

HSE was selected instead of the internal HSI RC oscillator because it provides a more stable and predictable local frequency reference. This matters when measuring intervals between GNSS PPS edges and timestamping IMU samples and impact events.

The clock tree produces a 216 MHz system clock. APB prescalers remain within the STM32F756 limits while providing suitable timer clocks.

HSE does not replace GNSS PPS. It improves short-term local timing stability between PPS edges, while PPS remains the external timing reference used to observe and correct local timer error.

The 8 MHz MCO clock connection is documented in [UM1974, Section 7.8.1 — OSC clock supply, page 26](https://www.st.com/resource/en/user_manual/dm00244518-stm32-nucleo-144-boards-mb1137-stmicroelectronics.pdf#page=26).


## HAL and FreeRTOS Time Bases

FreeRTOS uses SysTick, while the STM32 HAL time base uses TIM6.

Separating the time bases prevents the HAL tick and the FreeRTOS scheduler tick from competing for SysTick ownership.

TIM6 is therefore reserved for the HAL millisecond tick. A different timer will provide high-resolution event timestamps and GNSS PPS capture.


## Diagnostic Serial Interface

USART3 is used through the NUCLEO board's ST-LINK Virtual COM Port.

The initial interface is configured as 115200 baud, 8 data bits, no parity, and one stop bit. It provides startup messages, sensor test output, and fault diagnostics without requiring an external adapter.

The final logging architecture and synchronization of access to USART3 will be decided after the FreeRTOS task structure is defined.


## Code Generation Structure

STM32CubeMX generates peripheral initialization as separate `.c` and `.h` file pairs.

This keeps `main.c` small, separates generated initialization by responsibility, and makes the project easier to navigate and review. Application logic will be kept outside generated initialization code whenever practical.


## Open Design Topics

The following topics have not yet been finalized:

- IMU and GNSS Peripheral Assignment
- FreeRTOS Task and Communication Architecture
- IMU Sampling Rate and Filtering
- Impact Detection Algorithm and Thresholds
- GNSS NMEA Sentence Selection and Parsing
- Timer and Input-Capture Configuration for PPS
- High-Resolution Timestamp Representation
- PPS Synchronization and Clock-Correction Algorithm
- Logging Architecture
- Watchdog and Fault-Handling Strategy
- Event Storage and Host-Side Tools

These topics will receive their own sections after they are discussed and validated.
