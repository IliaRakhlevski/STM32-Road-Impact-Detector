/**
 * @file imu.h
 * @brief ICM-20948 IMU test interface.
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Raw ICM-20948 measurement associated with an IMU interrupt edge.
 */
typedef struct
{
    int16_t accelerometer_x;    /**< Raw acceleration along the X axis. */
    int16_t accelerometer_y;    /**< Raw acceleration along the Y axis. */
    int16_t accelerometer_z;    /**< Raw acceleration along the Z axis. */
    int16_t gyroscope_x;        /**< Raw angular velocity around the X axis. */
    int16_t gyroscope_y;        /**< Raw angular velocity around the Y axis. */
    int16_t gyroscope_z;        /**< Raw angular velocity around the Z axis. */
    int16_t temperature;        /**< Raw internal temperature sensor value. */
    uint32_t interrupt_capture; /**< TIM2 counter value captured on the IMU INT edge. */
    uint32_t interrupt_sequence; /**< Number of IMU INT edges captured since startup. */
    bool valid;                 /**< True when the measurement and timestamp are valid. */
} IMU_Data;

bool IMU_Init(void);

bool IMU_Test(void);

bool IMU_Read(IMU_Data *data);

void IMU_NotifyFromISR(void);

#endif /* IMU_H_ */
