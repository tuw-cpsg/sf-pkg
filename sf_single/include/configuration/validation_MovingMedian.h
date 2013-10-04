/**
 * @file 
 * @author Denise Ratasich
 * @date 02.10.2013
 *
 * @brief Validates the parameters for the MovingMedian.
 */

#ifndef __CONFIGURATION_VALIDATION_MOVINGMEDIAN_H__
#define __CONFIGURATION_VALIDATION_MOVINGMEDIAN_H__

// only 1 input is allowed
#if VECTOR_SIZE(TOPICS_IN) != 1
  #error "Validation of configuration header failed. MovingMedian: Only 1 input topic allowed."
#endif

// only 1 output is allowed
#if VECTOR_SIZE(TOPICS_OUT) != 1
  #error "Validation of configuration header failed. MovingMedian: Only 1 output topic allowed."
#endif

// window size
#ifdef WINDOW_SIZE
  #if WINDOW_SIZE > 0
  #else
    #error "Validation of configuration header failed. MovingMedian: Invalid window size."
  #endif
#endif

#endif // __CONFIGURATION_VALIDATION_MOVINGMEDIAN_H__
