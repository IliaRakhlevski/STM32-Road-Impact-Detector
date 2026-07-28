/**
 * @file time_capture.c
 * @brief Common hardware time capture implementation.
 */

#include "time_capture.h"

#include "imu.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

/** Latest GNSS PPS timestamp captured by TIM2 channel 1. */
static volatile uint32_t pps_capture;

/** Number of GNSS PPS rising edges captured since startup. */
static volatile uint32_t pps_sequence;

/** Latest IMU interrupt timestamp captured by TIM2 channel 2. */
static volatile uint32_t imu_capture;

/** Number of IMU interrupt rising edges captured since startup. */
static volatile uint32_t imu_sequence;

/**
 * @brief Start TIM2 input capture channels used by the application.
 *
 * @return true if both TIM2 input capture channels were started successfully;
 * otherwise false.
 */
bool TimeCapture_Init(void)
{
    if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    return (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) == HAL_OK);
}

/**
 * @brief Copy the latest GNSS PPS timestamp.
 *
 * @param pps Pointer to the destination structure.
 * @return true if at least one PPS edge has been captured; otherwise false.
 */
bool TimeCapture_GetPps(TimeCapture_Event *pps)
{
    if (pps == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    pps->capture = pps_capture;
    pps->sequence = pps_sequence;
    taskEXIT_CRITICAL();

    return (pps->sequence > 0U);
}

/**
 * @brief Copy the latest IMU interrupt timestamp.
 *
 * @param imu Pointer to the destination structure.
 * @return true if at least one IMU interrupt edge has been captured;
 * otherwise false.
 */
bool TimeCapture_GetImu(TimeCapture_Event *imu)
{
    if (imu == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    imu->capture = imu_capture;
    imu->sequence = imu_sequence;
    taskEXIT_CRITICAL();

    return (imu->sequence > 0U);
}

/**
 * @brief Handle a timer input capture event.
 *
 * TIM2 channel 1 captures the GNSS PPS rising edge. TIM2 channel 2 captures
 * the IMU interrupt rising edge.
 *
 * @param htim Pointer to the timer handle that generated the event.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == NULL) || (htim->Instance != TIM2))
    {
        return;
    }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        pps_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        pps_sequence++;
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        imu_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        imu_sequence++;
        IMU_NotifyFromISR();
    }
}
