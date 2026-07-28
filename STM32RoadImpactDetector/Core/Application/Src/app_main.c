/**
 * @file app_main.c
 * @brief Application initialization.
 */

#include "app_main.h"
#include "imu.h"
#include "time_capture.h"

#include <stdio.h>

/**
 * @brief Initialize the application.
 *
 * Prints the application startup message. Application tasks and
 * synchronization objects will be created here as the project evolves.
 */
void App_Init(void)
{
    printf("\r\nSTM32 Road Impact Detector\r\n");

    if (!TimeCapture_Init())
    {
        printf("Time capture initialization failed\r\n");
    }

    if (!IMU_Init())
    {
        printf("IMU task creation failed\r\n");
    }
}
