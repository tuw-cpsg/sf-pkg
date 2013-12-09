/**
 * @file 
 * @author Denise Ratasich
 * @date 09.12.2013
 *
 * @brief Example configuration UKF to filter a constant signal.
 *
 * Filters a single raw signal. It is assumed to be constant. Its the
 * simplest extended Kalman filter with a state and measurement vector
 * of size 1.
 */

// -----------------------------------------
// general
// -----------------------------------------

// Estimate every 100ms.
#define ESTIMATION_PERIOD	100

// -----------------------------------------
// input
// -----------------------------------------

// Constant signal but noisy.
#define TOPICS_IN				\
  ((signal, data, std_msgs::Float64))		\
  /**/

// No control input.

// Define the includes needed for the topics.
#include <std_msgs/Float64.h>

// -----------------------------------------
// output
// -----------------------------------------

#define TOPICS_OUT				\
  ((signal_estimated, 0))			\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

#define METHOD	UNSCENTED_KALMAN_FILTER

// required
#define STATE_TRANSITION_MODEL			\
  (x[0])					\
  /**/
#define OBSERVATION_MODEL			\
  (x[0])					\
  /**/
#define PROCESS_NOISE_COVARIANCE		\
  ( (0.2) )					\
  /**/
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (1) )					\
  /**/

// optional
#define INITIAL_STATE			(0)
#define INITIAL_ERROR_COVARIANCE	( (1) )
