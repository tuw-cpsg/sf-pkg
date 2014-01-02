/**
 * @file 
 * @author Denise Ratasich
 * @date 02.01.2014
 *
 * @brief Validates the parameters for ConfidenceWeightedAveraging.
 */

#ifndef __CONFIGURATION_VALIDATION_CONFIDENCEWEIGHTEDAVERAGING_H__
#define __CONFIGURATION_VALIDATION_CONFIDENCEWEIGHTEDAVERAGING_H__

// only 1 output is allowed
#if VECTOR_SIZE(TOPICS_OUT) != 1
  #error "Validation of configuration header failed. ConfidenceWeightedAveraging: Only 1 output topic allowed."
#endif

// IGNORE_ZERO_VARIANCE_VALUES is optional

#endif // __CONFIGURATION_VALIDATION_CONFIDENCEWEIGHTEDAVERAGING_H__
