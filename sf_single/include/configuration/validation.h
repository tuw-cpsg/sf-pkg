/**
 * @file 
 * @author Denise Ratasich
 * @date 02.10.2013
 *
 * @brief Validates the configuration header according to the chosen
 * method.
 */

#ifndef __CONFIGURATION_VALIDATION_H__
#define __CONFIGURATION_VALIDATION_H__

#include <boost/preprocessor/seq/size.hpp>
#include <boost/preprocessor/seq/elem.hpp>

// ----------------------------------------------------------------
// abbreviations for checking SIZES
// ----------------------------------------------------------------

#if METHOD == EXTENDED_KALMAN_FILTER  ||  METHOD == UNSCENTED_KALMAN_FILTER
#define STATE_SIZE		BOOST_PP_SEQ_SIZE(STATE_TRANSITION_MODEL)
#define MEASUREMENT_SIZE	BOOST_PP_SEQ_SIZE(OBSERVATION_MODEL)
#endif

/** 
 * @brief Expands to the number of elements in a sequence.
 */
#define VECTOR_SIZE(vector)	\
  BOOST_PP_SEQ_SIZE(vector)	\
  /**/

/** 
 * @brief Expands to the number of elements in a sequence, i.e. for a
 * sequence-of-sequences its the number of rows.
 */
#define MATRIX_ROWS(matrix)	\
  BOOST_PP_SEQ_SIZE(matrix)	\
  /**/

/** 
 * @brief Expands to the number of elements of the first sequence of a
 * sequence, i.e. for a sequence-of-sequences its the number of
 * columns.
 */
#define MATRIX_COLS(matrix)				\
  BOOST_PP_SEQ_SIZE(BOOST_PP_SEQ_ELEM(0,matrix))	\
  /**/

// ----------------------------------------------------------------
// validation general (things needed for every method)
// ----------------------------------------------------------------
#ifndef TOPICS
  #error "Validation failed. Topics, i.e. inputs, missing."
#endif

#ifndef METHOD
  #error "Validation failed. A method must be defined."
#else	// METHOD defined

// ----------------------------------------------------------------
// validation according to the method
// ----------------------------------------------------------------
#if METHOD == MOVING_MEDIAN
  #include "validation_MovingMedian.h"
#elif METHOD == MOVING_AVERAGE
  #include "validation_MovingAverage.h"
#elif METHOD == KALMAN_FILTER
  #include "validation_KalmanFilter.h"
#elif METHOD == EXTENDED_KALMAN_FILTER
  #include "validation_ExtendedKalmanFilter.h"
#elif METHOD == UNSCENTED_KALMAN_FILTER
  #include "validation_UnscentedKalmanFilter.h"
#else
  #error "Validation failed. Unknown method."
#endif

#endif	// METHOD defined

#endif // __CONFIGURATION_VALIDATION_H__
