/**
 * @file 
 * @author Denise Ratasich
 * @date 02.10.2013
 *
 * @brief Validates the parameters for the MovingAverage.
 */

#ifndef __CONFIGURATION_VALIDATION_MOVINGAVERAGE_H__
#define __CONFIGURATION_VALIDATION_MOVINGAVERAGE_H__

// only 1 input is allowed
#if VECTOR_SIZE(TOPICS) != 1
  #error "Validation of configuration header failed. MovingMedian: Only 1 topic (input entity) allowed."
#endif

// window size
#ifdef WINDOW_SIZE
  #if WINDOW_SIZE > 0
  #else
    #error "Validation of configuration header failed. MovingAverage: Invalid window size."
  #endif
#endif

// weighting coefficients (a vector, i.e. defined as a sequence)
#ifdef WEIGHTING_COEFFICIENTS
  #ifdef WINDOW_SIZE
    // check if window size matches the number of coefficients:
    // #coefficients = window size ... only b_k coefficients are passed
    // #coefficients = 2 * window size - 1 ... a_k coefficients are passed too
    #if !(VECTOR_SIZE(WEIGHTING_COEFFICIENTS == WINDOW_SIZE) ||	\
          VECTOR_SIZE(WEIGHTING_COEFFICIENTS) == 2*WINDOW_SIZE - 1)
      #error "Validation of configuration header failed. MovingAverage: Invalid number of coefficients."
    #endif
  #else 
    #error "Validation of configuration header failed. MovingAverage: Window size must be specified when weighting coefficients are given."
  #endif
#endif

#endif // __CONFIGURATION_VALIDATION_MOVINGAVERAGE_H__
