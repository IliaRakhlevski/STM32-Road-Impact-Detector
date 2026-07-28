/**
 * @file time_capture.h
 * @brief Common hardware time capture interface.
 */

#ifndef TIME_CAPTURE_H_
#define TIME_CAPTURE_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Timestamp and sequence number of a captured input event.
 */
typedef struct
{
    uint32_t capture;  /**< TIM2 counter value captured on the input edge. */
    uint32_t sequence; /**< Number of captured edges from this source. */
} TimeCapture_Event;

bool TimeCapture_Init(void);

bool TimeCapture_GetPps(TimeCapture_Event *pps);

bool TimeCapture_GetImu(TimeCapture_Event *imu);

#endif /* TIME_CAPTURE_H_ */
