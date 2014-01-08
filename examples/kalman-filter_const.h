/**
 * @file 
 * @author Denise Ratasich
 * @date 09.12.2013
 *
 * @brief Example configuration KF to filter a constant signal.
 *
 * Filters a single raw signal. It is assumed to be constant. Its the
 * simplest Kalman filter with a state and measurement vector of size
 * 1.
 */

// -----------------------------------------
// general
// -----------------------------------------

#define FILTER_PERIOD	(0.1)		// filter period

// -----------------------------------------
// input
// -----------------------------------------

// The noisy signal to estimate.
#define TOPICS_IN				\
  ((signal) (std_msgs::Float64) (data))		\
  /**/

// no control input

// Define the includes needed for these topics!
#include <std_msgs/Float64.h>

// -----------------------------------------
// output
// -----------------------------------------

// Filtered signal is published within "signal_estimated".
#define TOPICS_OUT				\
  ((signal_estimated) (0))			\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// Use a Kalman filter for estimation.
#define METHOD				KALMAN_FILTER

//
// required parameters
//

// Simply map the input to the output, there is no physical model
// behind it (it works like a moving average filter).
// 
// x_apriori[0] = x[0];
// 
#define STATE_TRANSITION_MODEL		( (1)   )

// Guess.
#define PROCESS_NOISE_COVARIANCE	( (0.2) )

// The input signal represents the state directly, e.g. no unit
// conversion.
// 
// x_0 = z_0
//
#define OBSERVATION_MODEL		( (1)   )

// Guess.
#define MEASUREMENT_NOISE_COVARIANCE	( (1)  )

//
// optional parameters
// 

//#define CONTROL_INPUT_MODEL		( (1) )

// Initialize state to 0.
#define INITIAL_STATE			(0)

// Initialize variance to 1.
#define INITIAL_ERROR_COVARIANCE	( (1) )

