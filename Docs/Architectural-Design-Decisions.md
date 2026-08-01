# Architectural Design Decisions

This document records the main architectural and design decisions of the STM32 Road Impact Detector project.

Its purpose is to explain **why** a solution was selected. Peripheral settings and implementation details belong in the source code and dedicated technical documentation.


## Project Scope

The system detects road impact candidates from ICM-20948 acceleration measurements and associates them with GNSS position and PPS-referenced UTC timestamps.

It does not attempt to classify an event as a pothole, speed bump, road joint, or another specific road feature. Reliable semantic classification would require additional sensing and processing, such as a camera and image analysis, which are outside the embedded focus of this project.

The project therefore concentrates on:

- Interrupt-Driven IMU Acceleration Acquisition
- GNSS Position and UTC Acquisition
- GNSS PPS and IMU Timestamp Capture
- PPS-Based Time Synchronization
- Acceleration-Baseline Impact Candidate Detection
- FreeRTOS Synchronization and Inter-Task Communication
- Automatic IMU I2C Communication Recovery


## Target Hardware

The target platform is the NUCLEO-F756ZG development board with an STM32F756ZGT6 microcontroller.

![NUCLEO-F756ZG development board](Images/nucleo-f756zg-board.jpeg)

The board provides sufficient processing and memory resources, multiple communication peripherals, 32-bit timers, an integrated ST-LINK debugger, and a Virtual COM Port. This makes it suitable for demonstrating a realistic RTOS-based embedded system without requiring a custom PCB.

The project uses real sensors rather than simulated data:

- Pimoroni ICM-20948 breakout board for motion acquisition
- Quectel LC86G-LA GNSS module on an HW-658 development board

![Pimoroni ICM-20948 breakout board](Images/icm-20948-breakout-front.jpg)

![Quectel LC86G-LA GNSS development board](Images/quectel-l80-gnss-board-front.jpg)

Additional views:

- [ICM-20948 breakout board — back](Images/icm-20948-breakout-back.jpg)
- [Quectel LC86G-LA GNSS development board — back](Images/quectel-l80-gnss-board-back.jpg)

Although the GNSS module shield is marked `L80-M39`, its boot output identifies
the installed receiver as `LC86G-LA` running firmware
`LC86GLANR01A03S` dated 2022-07-01. The firmware uses the Quectel PAIR command
protocol rather than the PMTK command set associated with the L80 family.

Real hardware exposes practical embedded-development problems that simulated input does not reproduce: device initialization, electrical interfaces, communication failures, asynchronous data arrival, sensor noise, and timing uncertainty.

Relevant documentation:

- [NUCLEO-F756ZG product page](https://www.st.com/en/evaluation-tools/nucleo-f756zg.html)
- [UM1974 — STM32 Nucleo-144 boards user manual](https://www.st.com/resource/en/user_manual/dm00244518-stm32-nucleo-144-boards-mb1137-stmicroelectronics.pdf)
- [STM32F756ZG product page](https://www.st.com/en/microcontrollers-microprocessors/stm32f756zg.html)
- [STM32F756xx datasheet](https://www.st.com/resource/en/datasheet/stm32f756zg.pdf)
- [RM0385 — STM32F75xxx/STM32F74xxx reference manual](https://www.st.com/resource/en/reference_manual/rm0385-stm32f75xxx-and-stm32f74xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [ICM-20948 product page](https://www.invensense.tdk.com/en-us/products/9-axis/icm-20948/)
- [DS-000189 — ICM-20948 datasheet](https://www.invensense.tdk.com/en-us/search-result?query=DS-000189%20%E2%80%93%20ICM-20948%20Datasheet)
- [Pimoroni ICM-20948 breakout board](https://shop.pimoroni.com/products/icm20948)
- [Quectel LC86G Series Hardware Design V1.1](https://quectel.com/content/uploads/2024/02/Quectel_LC86G_Series_Hardware_Design_V1.1.pdf)
- [Quectel LC86G Series GNSS Specification V1.1](https://www.quectel.com/content/uploads/sites/3/2022/12/Quectel_LC86G_Series_GNSS_Specification_V1.1.pdf)
- [Quectel LC26G(A&B), LC76G and LC86G Series GNSS Protocol Specification V1.1](https://quectel.com/content/uploads/2024/02/Quectel_LC26GABLC76GLC86G_Series_GNSS_Protocol_Specification_V1.1.pdf)


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

The 8 MHz HSE bypass clock supplied by the on-board ST-LINK MCO was selected instead of the STM32F756 internal 16 MHz HSI RC oscillator. The STM32F756xx datasheet specifies an HSI accuracy of +/-1% at 25 degrees Celsius and a larger possible variation over temperature. Avoiding this RC-oscillator tolerance provides a more suitable local frequency reference for measuring intervals between GNSS PPS edges and timestamping IMU samples and impact events.

The clock tree produces a 216 MHz system clock. APB prescalers remain within the STM32F756 limits while providing suitable timer clocks.

HSE does not replace GNSS PPS. It improves short-term local timing stability between PPS edges, while PPS remains the external timing reference used to observe and correct local timer error.

References:

- [STM32F756xx Datasheet](https://www.st.com/resource/en/datasheet/stm32f756zg.pdf), Section 6.3.10, Table 42, page 133: HSI oscillator characteristics and accuracy.
- [STM32 Nucleo-144 User Manual UM1974](https://www.st.com/resource/en/user_manual/dm00244518-nucleo-144-boards-mb1137-stmicroelectronics.pdf), Section 7.8.1, page 26: default 8 MHz ST-LINK MCO connection to the STM32 HSE input.


## HAL and FreeRTOS Time Bases

FreeRTOS uses SysTick, while the STM32 HAL time base uses TIM6.

Separating the time bases prevents the HAL tick and the FreeRTOS scheduler tick from competing for SysTick ownership.

TIM6 is therefore reserved for the HAL millisecond tick. A different timer will provide high-resolution event timestamps and GNSS PPS capture.


## Diagnostic Serial Interface

USART3 is used through the NUCLEO board's ST-LINK Virtual COM Port.

The initial interface is configured as 115200 baud, 8 data bits, no parity, and one stop bit. It provides startup messages, sensor test output, and fault diagnostics without requiring an external adapter.

Access to USART3 is protected by a FreeRTOS mutex. Each message is formatted in
a task-local buffer before the mutex is acquired, so complete lines from
different tasks cannot be mixed together.

Combined hardware testing also showed that continuous high-rate IMU logging can
delay application processing and interfere with timely handling of the GNSS DMA
stream. Routine IMU measurement output is therefore disabled. Detailed GNSS
diagnostics are retained temporarily while the GNSS and IMU integration is
being verified.


## GNSS PPS and NMEA Association

GNSS data is received through USART2 using circular DMA, while the PPS rising
edge is captured by TIM2 channel 1. TIM2 is a free-running 32-bit timer
configured at 1 MHz, providing a one-microsecond timestamp resolution.

Testing with a valid GNSS fix confirmed that PPS is generated once per second.
The start of each `$GNGGA` sentence was observed approximately 81–99 ms after
the associated PPS edge. The delay remained bounded rather than drifting over
time, demonstrating that PPS and NMEA output are derived from the same GNSS
time base.

No proprietary synchronization command is required during initialization.
PPS capture supplies the precise local timing reference, while the NMEA data
provides UTC time and position. The firmware will associate these two inputs
instead of relying on UART arrival time as an exact event timestamp.


## Sensor Task Structure

GNSS and IMU acquisition are handled by separate FreeRTOS tasks.

The GNSS task receives NMEA data through USART2 with circular DMA and parses the
GNGGA sentence used by the project. The resulting GNSS structure contains UTC,
position, fix validity, and the captured PPS timestamp.

The IMU task waits for a task notification generated by the ICM-20948 data-ready
interrupt. It then reads one accelerometer measurement through I²C. Blocking
I²C access is acceptable because only the IMU task owns the sensor interface
and each register transaction is short.

Native FreeRTOS task functions are used for application tasks. This keeps the
implementation consistent with the FreeRTOS programming model used during
development.


## Common GNSS and IMU Timestamp Capture

TIM2 is configured as a free-running 32-bit counter with a frequency of 1 MHz.
Its two input-capture channels provide timestamps from the same local time
base:

- Channel 1 captures the GNSS PPS rising edge.
- Channel 2 captures the rising edge from the ICM-20948 `INT1` data-ready
  output.

The common time-capture module owns the HAL timer callback and separates events
according to the active timer channel. Each captured event contains the timer
counter value and a monotonically increasing sequence number.

This arrangement allows an IMU measurement to be related to GNSS time without
using the execution time of a FreeRTOS task as the timestamp.


## IMU Data-Ready Interrupt and Acquisition Timing

The ICM-20948 is connected through I²C1. Its `RAW_DATA_0_RDY_EN` interrupt is
enabled so the sensor generates a pulse on the `INT1` pin whenever a new raw
measurement is ready. This signal is connected to TIM2 channel 2 and has an
output rate of approximately 102.3 Hz with the current sample-rate divider.

The rising edge on `INT1` causes TIM2 to capture its counter value in hardware.
The interrupt callback records this timestamp and notifies the IMU task. The
task subsequently reads the measurement registers through I²C and associates
the returned values with the captured interrupt timestamp.

Separating timestamp capture from register access is important because task
scheduling and the blocking I²C transaction introduce variable delay. The
execution time of the IMU task therefore does not determine the measurement
timestamp. Register access, calculations, and diagnostic output remain in task
context rather than being performed inside the interrupt callback.

The current implementation reads raw acceleration, angular velocity, and
internal temperature in one consecutive register transaction. Only
acceleration is required by the initial impact-detection algorithm. Gyroscope
data remains available for diagnostics and possible future filtering, but is
not part of the current detection decision.

The accelerometer full-scale setting is read from `ACCEL_CONFIG` during sensor
initialization. The firmware derives the selected range from `ACCEL_FS_SEL`
instead of relying on a fixed sensitivity constant. Raw acceleration values
are then converted to units of g for use by the impact-detection algorithm.

The accelerometer sample-rate divider is 10. With the 1.125 kHz accelerometer
base rate, this produces approximately 102 samples per second.


## Impact Candidate Detection

The detector operates on the acceleration vector magnitude rather than on an
individual sensor axis. The magnitude is calculated as:

`|a| = sqrt(ax² + ay² + az²)`

This makes the initial detector independent of the orientation of the IMU axes
relative to the vehicle. A stationary sensor measures a magnitude close to 1 g
because gravity is included.

### Initial Baseline

The first ten valid IMU samples are averaged to establish the initial
acceleration baseline. Candidate detection does not run until this first
baseline is available. Consequently, the earliest sample that can open a
candidate is the eleventh valid sample after initialization.

These first ten samples are assumed to represent normal operation; the
firmware cannot verify this before a reference exists. Starting the system
during a strong disturbance can therefore bias the initial baseline. A future
startup-calibration improvement may require several stable groups before the
baseline is accepted.

### Per-Sample Detection

After initialization, every individual acceleration sample is compared with
the current baseline:

`deviation = abs(current magnitude - baseline)`

The absolute value allows both positive and negative changes in acceleration
magnitude to be detected. A deviation of at least 0.20 g opens an impact
candidate immediately, even if only one sample crosses the threshold. This
preserves sensitivity to short impacts that may occupy only one sample at the
approximately 102.3 Hz IMU output rate.

An active candidate may contain several threshold crossings. The sample with
the greatest absolute deviation is retained as the candidate peak. Its
acceleration magnitude, reference baseline, TIM2 capture and IMU interrupt
sequence become the representative event data.

Any threshold crossing resets the count of normal samples following the
candidate peak. The candidate is completed only after three consecutive
samples fall below the threshold. At approximately 102.3 samples per second,
this represents about 29 ms of continuous return toward the baseline. The
three-sample requirement prevents a brief dip below the threshold from
splitting one oscillating disturbance into multiple candidates.

### Baseline Update

Samples are added to the baseline calculation only while no candidate is
active. This prevents the disturbance from moving the reference toward itself.
When a new candidate opens, any partially collected normal group is discarded.

Ten eligible samples form one baseline update group. The group average does
not replace the baseline directly; the baseline moves 10% toward it:

`new baseline = old baseline + 0.10 × (group average - old baseline)`

This low-pass update follows slow changes such as vehicle attitude while
remaining relatively insensitive to short disturbances. When the third
consecutive below-threshold sample completes a candidate, that same sample
becomes the first member of the next normal baseline group.

### Candidate Meaning and Limitations

The output deliberately uses the term *impact candidate*. Acceleration alone
cannot reliably classify a pothole, speed bump, road joint, or another source
of vehicle motion. A single noisy sample can also open a candidate because the
current algorithm does not require multiple above-threshold samples.

The 0.20 g threshold, ten-sample averaging group, 0.10 baseline update factor
and three-sample completion rule are experimental parameters. They must be
validated together in controlled road tests because changing any one of them
affects sensitivity, response time and false-positive behavior.


## GNSS and IMU Data Association

Each valid GNGGA sentence is associated with a captured PPS edge. The PPS edge
provides the precise local second boundary, while GNGGA supplies UTC and
position. An IMU timestamp is compared with the relevant PPS capture, and the
microsecond offset from that boundary is added to UTC.

An impact candidate therefore contains:

- The peak acceleration magnitude
- The baseline and deviation
- The event timestamp from the common TIM2 time base
- UTC derived from GNSS PPS and GNGGA
- The latest associated GNSS position

If no valid GNSS synchronization is available, the event remains usable with a
local TIM2 timestamp and is explicitly reported without GNSS data.


## Combined Hardware Validation

Outdoor testing confirmed simultaneous operation of the complete acquisition
path:

- Valid GNGGA position and UTC were received continuously.
- TIM2 captured one GNSS PPS edge per second.
- The ICM-20948 produced approximately 102 measurements per second.
- Controlled movement produced impact candidates containing UTC and position.

The tests did not indicate physical interference from the IMU module. The
earlier GNSS failure during combined operation was isolated to excessive
blocking serial diagnostics: suppressing routine high-rate IMU output restored
stable simultaneous GNSS, PPS, and impact-candidate processing.


## IMU Communication Recovery

Mechanical prototypes can produce temporary I²C contact failures. When an IMU
read fails, the IMU task resets and reinitializes the STM32 I²C peripheral,
detects the ICM-20948 again through its address and `WHO_AM_I` register, and
restores the sensor configuration.

If communication is still unavailable, the task retries after a delay instead
of requiring a complete STM32 reset. This provides basic runtime fault recovery
while preserving a straightforward implementation. Reliable soldered
connections remain a hardware requirement for final testing.


## Code Generation Structure

STM32CubeMX generates peripheral initialization as separate `.c` and `.h` file pairs.

This keeps `main.c` small, separates generated initialization by responsibility, and makes the project easier to navigate and review. Application logic will be kept outside generated initialization code whenever practical.


## Open Design Topics

The following topics have not yet been finalized:

- Controlled Threshold Validation and False-Positive Reduction
- Additional IMU Filtering
- TIM2 Frequency-Error Measurement and Correction Using GNSS PPS
- Compact Event Queueing and Host Reporting
- Production Logging Level
- Watchdog and Fault-Handling Strategy
- Event Storage and Host-Side Tools

These topics will receive their own sections after they are discussed and validated.
