/**
 * @file impact_detector.h
 * @brief Road impact candidate detection interface.
 */

#ifndef IMPACT_DETECTOR_H_
#define IMPACT_DETECTOR_H_

#include "gnss.h"
#include "imu.h"

/**
 * @brief Completed road impact candidate with synchronized position and time.
 */
typedef struct
{
    float peak_acceleration_g;      /**< Acceleration magnitude at the strongest deviation. */
    float reference_average_g;      /**< Reference average used for candidate detection. */
    float peak_deviation_g;         /**< Maximum deviation from the reference average. */
    uint32_t imu_capture;           /**< TIM2 counter value captured at the candidate peak. */
    uint32_t imu_sequence;          /**< IMU interrupt sequence number at the candidate peak. */
    GNSS_Fix gnss_fix;              /**< GNSS position and UTC second associated with the peak. */
    uint32_t time_after_pps_us;     /**< Candidate peak time after the associated PPS edge. */
    bool time_valid;                /**< True when GNSS and PPS synchronization is available. */
    bool valid;                     /**< True when the candidate structure contains an event. */
} ImpactCandidate;

void ImpactDetector_Init(void);

void ImpactDetector_ProcessSample(const IMU_Data *data);

bool ImpactDetector_GetLatestCandidate(ImpactCandidate *candidate);

#endif /* IMPACT_DETECTOR_H_ */
