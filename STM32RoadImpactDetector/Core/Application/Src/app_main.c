/**
 * @file app_main.c
 * @brief Application initialization.
 */

#include "app_main.h"
#include "console.h"
#include "gnss.h"
#include "impact_detector.h"
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
    if (!Console_Init())
    {
        Console_Printf("Console mutex creation failed\r\n");
    }

    Console_Printf("\r\nSTM32 Road Impact Detector\r\n");

    if (!TimeCapture_Init())
    {
        Console_Printf("Time capture initialization failed\r\n");
    }

    ImpactDetector_Init();

    if (!GNSS_Init())
    {
        Console_Printf("GNSS task creation failed\r\n");
    }

    if (!IMU_Init())
    {
        Console_Printf("IMU task creation failed\r\n");
    }
}
