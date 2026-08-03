/**
 * @file impact_detector_config.h
 * @brief Road impact candidate detection configuration.
 */

#ifndef IMPACT_DETECTOR_CONFIG_H_
#define IMPACT_DETECTOR_CONFIG_H_

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

#endif /* IMPACT_DETECTOR_CONFIG_H_ */
