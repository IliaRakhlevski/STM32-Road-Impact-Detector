/**
 * @file gnss.h
 * @brief GNSS module interface.
 */

#ifndef GNSS_H_
#define GNSS_H_

#include <stdbool.h>
#include <stdint.h>

/** Maximum length of the UTC time field received in a GGA sentence. */
#define GNSS_UTC_TIME_SIZE 11U

/** Maximum length of a latitude or longitude field. */
#define GNSS_COORDINATE_SIZE 16U

/**
 * @brief GNSS position associated with a captured PPS edge.
 */
typedef struct
{
    char latitude[GNSS_COORDINATE_SIZE];   /**< Latitude in NMEA ddmm.mmmm format. */
    char longitude[GNSS_COORDINATE_SIZE];  /**< Longitude in NMEA dddmm.mmmm format. */
    char latitude_direction;               /**< Latitude hemisphere: 'N' or 'S'. */
    char longitude_direction;              /**< Longitude hemisphere: 'E' or 'W'. */
    char utc_time[GNSS_UTC_TIME_SIZE];      /**< UTC time in NMEA hhmmss.sss format. */
    uint32_t pps_capture;                   /**< TIM2 counter value captured on the PPS edge. */
    uint32_t pps_sequence;                  /**< Number of PPS edges captured since startup. */
    bool valid;                             /**< True when the position and PPS data are valid. */
} GNSS_Fix;

bool GNSS_Init(void);

bool GNSS_GetLatestFix(GNSS_Fix *fix);

#endif /* GNSS_H_ */
