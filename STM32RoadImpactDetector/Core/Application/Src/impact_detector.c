/**
 * @file impact_detector.c
 * @brief Road impact candidate detection implementation.
 */

#include "impact_detector.h"

#include "console.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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

/** TIM2 capture used for the most recent periodic status message. */
static uint32_t impact_detector_last_status_capture;

/**
 * @brief Format UTC time with the measured offset after PPS.
 *
 * NMEA GGA supplies UTC as hhmmss.sss, while the common TIM2 time base
 * measures how many microseconds the IMU event occurred after the associated
 * PPS second boundary. The function combines both values and normalizes any
 * carry from microseconds to seconds, minutes and hours.
 *
 * Example: UTC 123519.250 plus an offset of 900000 us becomes
 * 12:35:20.150000.
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

    /* Convert the fixed ASCII digit positions in hhmmss to integer fields. */
    hours = ((uint32_t)(utc_time[0] - '0') * 10U) + (uint32_t)(utc_time[1] - '0');
    minutes = ((uint32_t)(utc_time[2] - '0') * 10U) + (uint32_t)(utc_time[3] - '0');
    seconds = ((uint32_t)(utc_time[4] - '0') * 10U) + (uint32_t)(utc_time[5] - '0');
    microseconds = offset_us;

    /*
     * Convert the three NMEA fractional digits from milliseconds to
     * microseconds and add them to the measured offset.
     */
    if (utc_time[6] == '.')
    {
        microseconds += (uint32_t)(utc_time[7] - '0') * 100000U;
        microseconds += (uint32_t)(utc_time[8] - '0') * 10000U;
        microseconds += (uint32_t)(utc_time[9] - '0') * 1000U;
    }

    /*
     * Propagate overflow through the time fields. Modulo 24 keeps the result
     * in one UTC day if the addition crosses midnight.
     */
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
 * @brief Convert one NMEA coordinate to decimal degrees.
 *
 * NMEA represents latitude as ddmm.mmmm and longitude as dddmm.mmmm: the
 * digits before the final two integer digits of the numeric part are degrees,
 * and the remainder is minutes. Decimal degrees are calculated as:
 *
 *     decimal degrees = degrees + minutes / 60
 *
 * Example: 3149.334166 means 31 degrees and 49.334166 minutes, or
 * 31 + 49.334166 / 60 = 31.822236 degrees.
 *
 * This function returns the positive coordinate magnitude. The N/S/E/W
 * direction remains separate and is printed by the caller rather than being
 * converted to a negative sign for S or W.
 *
 * @param coordinate Coordinate in ddmm.mmmm or dddmm.mmmm format.
 * @param direction Coordinate direction: N, S, E or W.
 * @param decimal_degrees Destination for the converted coordinate magnitude.
 * @return true if the coordinate was converted successfully; otherwise false.
 */
static bool ImpactDetector_ConvertCoordinate(const char *coordinate, char direction, float *decimal_degrees)
{
    float coordinate_value;
    float minutes;
    uint32_t degrees;

    if ((coordinate == NULL) || (coordinate[0] == '\0') || (decimal_degrees == NULL))
    {
        return false;
    }

    /*
     * Dividing by 100 and truncating removes the final two minute digits and
     * leaves the degree part for either latitude or longitude.
     */
    coordinate_value = strtof(coordinate, NULL);
    degrees = (uint32_t)(coordinate_value / 100.0f);
    minutes = coordinate_value - ((float)degrees * 100.0f);
    *decimal_degrees = (float)degrees + (minutes / 60.0f);

    if ((direction != 'N') && (direction != 'S') && (direction != 'E') && (direction != 'W'))
    {
        return false;
    }

    return true;
}

/**
 * @brief Print the periodic combined GNSS and IMU status.
 *
 * @param capture TIM2 capture associated with the IMU average.
 * @param average_acceleration Average acceleration magnitude in g.
 */
static void ImpactDetector_PrintStatus(uint32_t capture, float average_acceleration)
{
    char formatted_utc_time[16];
    float latitude_degrees;
    float longitude_degrees;
    GNSS_Fix fix;
    uint32_t time_after_pps_us;

    if (GNSS_GetFixForCapture(capture, &fix, &time_after_pps_us) &&
        ImpactDetector_ConvertCoordinate(fix.latitude, fix.latitude_direction, &latitude_degrees) &&
        ImpactDetector_ConvertCoordinate(fix.longitude, fix.longitude_direction, &longitude_degrees))
    {
        ImpactDetector_FormatUtcTime(fix.utc_time, time_after_pps_us, formatted_utc_time, sizeof(formatted_utc_time));
        Console_Printf("STATUS: UTC=%.8s, LAT=%.6f deg %c, LON=%.6f deg %c, IMU_AVG=%.2f g\r\n",
                       formatted_utc_time,
                       latitude_degrees,
                       fix.latitude_direction,
                       longitude_degrees,
                       fix.longitude_direction,
                       average_acceleration);
    }
    else
    {
        Console_Printf("STATUS: TIME=%lu.%06lu s, GNSS=UNAVAILABLE, IMU_AVG=%.2f g\r\n",
                       (unsigned long)(capture / IMPACT_DETECTOR_TIMER_TICKS_PER_SECOND),
                       (unsigned long)(capture % IMPACT_DETECTOR_TIMER_TICKS_PER_SECOND),
                       average_acceleration);
    }
}

/**
 * @brief Complete the current candidate and add synchronized GNSS data.
 */
static void ImpactDetector_CompleteCandidate(void)
{
    char formatted_utc_time[16];
    float latitude_degrees;
    float longitude_degrees;

    impact_detector_current_candidate.time_valid =
        GNSS_GetFixForCapture(impact_detector_current_candidate.imu_capture,
                              &impact_detector_current_candidate.gnss_fix,
                              &impact_detector_current_candidate.time_after_pps_us);
    impact_detector_current_candidate.valid = true;
    impact_detector_latest_candidate = impact_detector_current_candidate;

    if (impact_detector_current_candidate.time_valid &&
        ImpactDetector_ConvertCoordinate(impact_detector_current_candidate.gnss_fix.latitude,
                                         impact_detector_current_candidate.gnss_fix.latitude_direction,
                                         &latitude_degrees) &&
        ImpactDetector_ConvertCoordinate(impact_detector_current_candidate.gnss_fix.longitude,
                                         impact_detector_current_candidate.gnss_fix.longitude_direction,
                                         &longitude_degrees))
    {
        ImpactDetector_FormatUtcTime(impact_detector_current_candidate.gnss_fix.utc_time,
                                     impact_detector_current_candidate.time_after_pps_us,
                                     formatted_utc_time,
                                     sizeof(formatted_utc_time));

		Console_Printf("*** IMPACT CANDIDATE: UTC=%s, LAT=%.6f deg %c, LON=%.6f deg %c\r\n    ACC=%.2f g, BASELINE=%.2f g, DEVIATION=%.2f g ***\r\n",
                       formatted_utc_time,
                       latitude_degrees,
                       impact_detector_current_candidate.gnss_fix.latitude_direction,
                       longitude_degrees,
                       impact_detector_current_candidate.gnss_fix.longitude_direction,
                       impact_detector_current_candidate.peak_acceleration_g,
                       impact_detector_current_candidate.reference_average_g,
                       impact_detector_current_candidate.peak_deviation_g);
    }
    else
    {
		Console_Printf("*** IMPACT CANDIDATE: TIME=%lu.%06lu s, ACC=%.2f g, BASELINE=%.2f g\r\n    DEVIATION=%.2f g, GNSS=UNAVAILABLE ***\r\n",
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
    impact_detector_last_status_capture = 0U;
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
 * Processing has two stages:
 * 1. Compare the current acceleration magnitude with the established baseline
 *    and start, update or complete an impact candidate.
 * 2. If no candidate is active, collect the sample into the next normal group
 *    and use each completed group to update the slowly changing baseline.
 *
 * Samples that belong to an active candidate are deliberately excluded from
 * the baseline so the disturbance cannot pull the reference toward itself.
 *
 * @param data Pointer to the IMU measurement.
 */
void ImpactDetector_ProcessSample(const IMU_Data *data)
{
    float group_average_acceleration;
    float deviation;

    if ((data == NULL) || !data->valid)
    {
        return;
    }

    /*
     * The first complete normal group establishes the initial baseline. Until
     * then there is no reference against which the current sample can be
     * classified, so candidate detection is skipped.
     */
    if (impact_detector_average_valid)
    {
        /*
         * Use the absolute difference so both an increase and a decrease in
         * acceleration magnitude can form an impact candidate.
         */
        deviation = fabsf(data->acceleration_magnitude_g - impact_detector_average_acceleration);

        if (deviation >= IMPACT_DETECTOR_THRESHOLD_G)
        {
            /*
             * The first threshold crossing starts a new candidate. Discard
             * the partially collected normal group so pre-impact and
             * post-impact samples cannot be averaged together.
             */
            if (!impact_detector_candidate_active)
            {
                memset(&impact_detector_current_candidate, 0, sizeof(impact_detector_current_candidate));
                impact_detector_candidate_active = true;
                impact_detector_acceleration_sum = 0.0f;
                impact_detector_sample_count = 0U;
            }

            /*
             * Any new threshold crossing means the disturbance is still in
             * progress, so restart the count of consecutive normal samples.
             */
            impact_detector_normal_sample_count = 0U;

            /*
             * A candidate may contain several abnormal samples. Keep only the
             * sample with the largest absolute deviation; its acceleration
             * and captured TIM2 timestamp represent the candidate event.
             */
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
            /*
             * Do not finish a candidate after only one normal sample because
             * a multi-sample impact can briefly cross back below the
             * threshold. Require several consecutive normal measurements.
             */
            impact_detector_normal_sample_count++;

            if (impact_detector_normal_sample_count == IMPACT_DETECTOR_END_SAMPLE_COUNT)
            {
                ImpactDetector_CompleteCandidate();
            }
        }
    }

    /*
     * Update the baseline only when no candidate is active. If the current
     * sample is the final required normal sample, CompleteCandidate() clears
     * the active flag above and this same sample starts the next normal group.
     */
    if (!impact_detector_candidate_active)
    {
        impact_detector_acceleration_sum += data->acceleration_magnitude_g;
        impact_detector_sample_count++;

        if (impact_detector_sample_count == IMPACT_DETECTOR_AVERAGE_SAMPLE_COUNT)
        {
            /* Average one group of normal samples to suppress sample noise. */
            group_average_acceleration = impact_detector_acceleration_sum / IMPACT_DETECTOR_AVERAGE_SAMPLE_COUNT;

            if (!impact_detector_average_valid)
            {
                /* The first complete group becomes the initial baseline. */
                impact_detector_average_acceleration = group_average_acceleration;
                impact_detector_average_valid = true;
            }
            else
            {
                /*
                 * Move the baseline only 10% toward the latest group average.
                 * This tracks slow changes such as vehicle attitude while
                 * remaining insensitive to short acceleration disturbances.
                 */
                impact_detector_average_acceleration += IMPACT_DETECTOR_BASELINE_UPDATE_FACTOR *
                                                        (group_average_acceleration - impact_detector_average_acceleration);
            }

            /* Print at most one combined GNSS/IMU status message per second. */
            if ((uint32_t)(data->interrupt_capture - impact_detector_last_status_capture) >= IMPACT_DETECTOR_TIMER_TICKS_PER_SECOND)
            {
                ImpactDetector_PrintStatus(data->interrupt_capture, impact_detector_average_acceleration);
                impact_detector_last_status_capture = data->interrupt_capture;
            }

            /* Start collecting the next group of normal measurements. */
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
