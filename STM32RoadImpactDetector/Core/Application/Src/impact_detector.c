/**
 * @file impact_detector.c
 * @brief Road impact candidate detection implementation.
 */

#include "impact_detector.h"

#include "console.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/** Number of acceleration measurements used for one average value. */
#define IMPACT_DETECTOR_AVERAGE_SAMPLE_COUNT 10U

/** Experimental acceleration deviation threshold in g. */
#define IMPACT_DETECTOR_THRESHOLD_G 0.20f

/** Part of the new normal average used to update the baseline. */
#define IMPACT_DETECTOR_BASELINE_UPDATE_FACTOR 0.10f

/** Number of normal measurements required to finish one candidate. */
#define IMPACT_DETECTOR_END_SAMPLE_COUNT 3U

/** Number of TIM2 counter ticks in one second. */
#define IMPACT_DETECTOR_TIMER_TICKS_PER_SECOND 1000000U

/** Sum of acceleration magnitudes in the current averaging group. */
static float impact_detector_acceleration_sum;

/** Slowly updated normal acceleration baseline. */
static float impact_detector_average_acceleration;

/** Number of measurements collected in the current averaging group. */
static uint32_t impact_detector_sample_count;

/** Indicates that the first average acceleration value is available. */
static bool impact_detector_average_valid;

/** Indicates that a sequence of threshold crossings is being collected. */
static bool impact_detector_candidate_active;

/** Number of normal measurements received after the last threshold crossing. */
static uint32_t impact_detector_normal_sample_count;

/** Candidate currently being collected. */
static ImpactCandidate impact_detector_current_candidate;

/** Latest completed impact candidate. */
static ImpactCandidate impact_detector_latest_candidate;

/**
 * @brief Format UTC time with the measured offset after PPS.
 *
 * @param utc_time UTC time in NMEA hhmmss.sss format.
 * @param offset_us Measured time after PPS in microseconds.
 * @param formatted_time Destination string for the formatted UTC time.
 * @param formatted_time_size Size of the destination string.
 */
static void ImpactDetector_FormatUtcTime(const char *utc_time, uint32_t offset_us, char *formatted_time, size_t formatted_time_size)
{
    uint32_t hours;
    uint32_t minutes;
    uint32_t seconds;
    uint32_t microseconds;

    hours = ((uint32_t)(utc_time[0] - '0') * 10U) + (uint32_t)(utc_time[1] - '0');
    minutes = ((uint32_t)(utc_time[2] - '0') * 10U) + (uint32_t)(utc_time[3] - '0');
    seconds = ((uint32_t)(utc_time[4] - '0') * 10U) + (uint32_t)(utc_time[5] - '0');
    microseconds = offset_us;

    if (utc_time[6] == '.')
    {
        microseconds += (uint32_t)(utc_time[7] - '0') * 100000U;
        microseconds += (uint32_t)(utc_time[8] - '0') * 10000U;
        microseconds += (uint32_t)(utc_time[9] - '0') * 1000U;
    }

    seconds += microseconds / IMPACT_DETECTOR_TIMER_TICKS_PER_SECOND;
    microseconds %= IMPACT_DETECTOR_TIMER_TICKS_PER_SECOND;
    minutes += seconds / 60U;
    seconds %= 60U;
    hours += minutes / 60U;
    minutes %= 60U;
    hours %= 24U;

    snprintf(formatted_time,
             formatted_time_size,
             "%02lu:%02lu:%02lu.%06lu",
             (unsigned long)hours,
             (unsigned long)minutes,
             (unsigned long)seconds,
             (unsigned long)microseconds);
}

/**
 * @brief Complete the current candidate and add synchronized GNSS data.
 */
static void ImpactDetector_CompleteCandidate(void)
{
    char formatted_utc_time[16];

    impact_detector_current_candidate.time_valid =
        GNSS_GetFixForCapture(impact_detector_current_candidate.imu_capture,
                              &impact_detector_current_candidate.gnss_fix,
                              &impact_detector_current_candidate.time_after_pps_us);
    impact_detector_current_candidate.valid = true;
    impact_detector_latest_candidate = impact_detector_current_candidate;

    if (impact_detector_current_candidate.time_valid)
    {
        ImpactDetector_FormatUtcTime(impact_detector_current_candidate.gnss_fix.utc_time,
                                     impact_detector_current_candidate.time_after_pps_us,
                                     formatted_utc_time,
                                     sizeof(formatted_utc_time));

        Console_Printf("IMPACT CANDIDATE: UTC=%s, ACC=%.2f g, BASELINE=%.2f g, DEV=%.2f g, POSITION=%s %c, %s %c\r\n",
                       formatted_utc_time,
                       impact_detector_current_candidate.peak_acceleration_g,
                       impact_detector_current_candidate.reference_average_g,
                       impact_detector_current_candidate.peak_deviation_g,
                       impact_detector_current_candidate.gnss_fix.latitude,
                       impact_detector_current_candidate.gnss_fix.latitude_direction,
                       impact_detector_current_candidate.gnss_fix.longitude,
                       impact_detector_current_candidate.gnss_fix.longitude_direction);
    }
    else
    {
        Console_Printf("IMPACT CANDIDATE: TIME=%lu.%06lu s, ACC=%.2f g, BASELINE=%.2f g, DEV=%.2f g, GNSS unavailable\r\n",
                       (unsigned long)(impact_detector_current_candidate.imu_capture / IMPACT_DETECTOR_TIMER_TICKS_PER_SECOND),
                       (unsigned long)(impact_detector_current_candidate.imu_capture % IMPACT_DETECTOR_TIMER_TICKS_PER_SECOND),
                       impact_detector_current_candidate.peak_acceleration_g,
                       impact_detector_current_candidate.reference_average_g,
                       impact_detector_current_candidate.peak_deviation_g);
    }

    memset(&impact_detector_current_candidate, 0, sizeof(impact_detector_current_candidate));
    impact_detector_candidate_active = false;
    impact_detector_normal_sample_count = 0U;
}

/**
 * @brief Initialize impact candidate detection state.
 */
void ImpactDetector_Init(void)
{
    impact_detector_acceleration_sum = 0.0f;
    impact_detector_average_acceleration = 0.0f;
    impact_detector_sample_count = 0U;
    impact_detector_average_valid = false;
    impact_detector_candidate_active = false;
    impact_detector_normal_sample_count = 0U;
    memset(&impact_detector_current_candidate, 0, sizeof(impact_detector_current_candidate));
    memset(&impact_detector_latest_candidate, 0, sizeof(impact_detector_latest_candidate));
}

/**
 * @brief Process one timestamped IMU measurement.
 *
 * Each measurement is compared with the average of the previous group before
 * it is added to the next group. This allows a short acceleration change to be
 * detected without waiting for the next average value.
 *
 * @param data Pointer to the IMU measurement.
 */
void ImpactDetector_ProcessSample(const IMU_Data *data)
{
    char formatted_utc_time[16];
    float group_average_acceleration;
    float deviation;
    GNSS_Fix average_fix;
    uint32_t time_after_pps_us;

    if ((data == NULL) || !data->valid)
    {
        return;
    }

    if (impact_detector_average_valid)
    {
        deviation = fabsf(data->acceleration_magnitude_g - impact_detector_average_acceleration);

        if (deviation >= IMPACT_DETECTOR_THRESHOLD_G)
        {
            if (!impact_detector_candidate_active)
            {
                memset(&impact_detector_current_candidate, 0, sizeof(impact_detector_current_candidate));
                impact_detector_candidate_active = true;
                impact_detector_acceleration_sum = 0.0f;
                impact_detector_sample_count = 0U;
            }

            impact_detector_normal_sample_count = 0U;

            if (deviation > impact_detector_current_candidate.peak_deviation_g)
            {
                impact_detector_current_candidate.peak_acceleration_g = data->acceleration_magnitude_g;
                impact_detector_current_candidate.reference_average_g = impact_detector_average_acceleration;
                impact_detector_current_candidate.peak_deviation_g = deviation;
                impact_detector_current_candidate.imu_capture = data->interrupt_capture;
                impact_detector_current_candidate.imu_sequence = data->interrupt_sequence;
            }
        }
        else if (impact_detector_candidate_active)
        {
            impact_detector_normal_sample_count++;

            if (impact_detector_normal_sample_count == IMPACT_DETECTOR_END_SAMPLE_COUNT)
            {
                ImpactDetector_CompleteCandidate();
            }
        }
    }

    if (!impact_detector_candidate_active)
    {
        impact_detector_acceleration_sum += data->acceleration_magnitude_g;
        impact_detector_sample_count++;

        if (impact_detector_sample_count == IMPACT_DETECTOR_AVERAGE_SAMPLE_COUNT)
        {
            group_average_acceleration = impact_detector_acceleration_sum / IMPACT_DETECTOR_AVERAGE_SAMPLE_COUNT;

            if (!impact_detector_average_valid)
            {
                impact_detector_average_acceleration = group_average_acceleration;
                impact_detector_average_valid = true;
            }
            else
            {
                impact_detector_average_acceleration += IMPACT_DETECTOR_BASELINE_UPDATE_FACTOR *
                                                        (group_average_acceleration - impact_detector_average_acceleration);
            }

            if (GNSS_GetFixForCapture(data->interrupt_capture, &average_fix, &time_after_pps_us))
            {
                ImpactDetector_FormatUtcTime(average_fix.utc_time,
                                             time_after_pps_us,
                                             formatted_utc_time,
                                             sizeof(formatted_utc_time));

                /* Console_Printf("IMU: UTC=%s, AVG=%.2f g\r\n", formatted_utc_time, group_average_acceleration); */
            }
            else
            {
                /*
                Console_Printf("IMU: TIME=%lu.%06lu s, AVG=%.2f g, GNSS unavailable\r\n",
                               (unsigned long)(data->interrupt_capture / IMPACT_DETECTOR_TIMER_TICKS_PER_SECOND),
                               (unsigned long)(data->interrupt_capture % IMPACT_DETECTOR_TIMER_TICKS_PER_SECOND),
                               group_average_acceleration);
                */
            }

            impact_detector_acceleration_sum = 0.0f;
            impact_detector_sample_count = 0U;
        }
    }
}

/**
 * @brief Copy the latest completed impact candidate.
 *
 * @param candidate Pointer to the destination structure.
 * @return true if a completed candidate is available; otherwise false.
 */
bool ImpactDetector_GetLatestCandidate(ImpactCandidate *candidate)
{
    if (candidate == NULL)
    {
        return false;
    }

    *candidate = impact_detector_latest_candidate;
    return candidate->valid;
}
