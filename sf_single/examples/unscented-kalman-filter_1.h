/**
 * @file 
 * @author Denise Ratasich
 * @date 19.09.2013
 *
 * @brief Example configuration UKF 1.
 */

// -----------------------------------------
// input
// -----------------------------------------

// List input topics here, format (no commas needed between topic
// tuples): ((name, field, type)).
#define TOPICS_IN				\
  ((signal, data, std_msgs::Float64))		\
  /**/

// Define the includes needed for these topics!
#include <std_msgs/Float64.h>

// -----------------------------------------
// output
// -----------------------------------------

#define TOPICS_OUT				\
  ((state_0_fused, 0))				\
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
  ( (0.1) )
  /**/
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (10) )
  /**/

// optional
//#define CONTROL_INPUT
//#define INITIAL_STATE
//#define INITIAL_ERROR_COVARIANCE
