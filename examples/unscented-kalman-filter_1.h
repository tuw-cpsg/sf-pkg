/**
 * @file 
 * @author Denise Ratasich
 * @date 19.09.2013
 *
 * @brief Example configuration UKF 1.
 *
 * Model a constant signal with the possibility to add an offset.
 */

// -----------------------------------------
// general
// -----------------------------------------

// Estimate every 50ms.
#define ESTIMATION_PERIOD	50

// -----------------------------------------
// input
// -----------------------------------------

// Constant signal but noisy.
#define TOPICS_IN				\
  ((/signal, data, std_msgs::Float64))		\
  /**/

// Additional offsets for the signal.
#define TOPICS_IN_CTRL				\
  ((/ctrl1, data, std_msgs::Float64))		\
  ((/ctrl2, data, std_msgs::Float64))		\
  /**/

// Define the includes needed for the topics.
#include <std_msgs/Float64.h>

// -----------------------------------------
// output
// -----------------------------------------

#define TOPICS_OUT				\
  ((state_0, 0))				\
  ((state_1, 1))				\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

#define METHOD	UNSCENTED_KALMAN_FILTER

// required
#define STATE_TRANSITION_MODEL			\
  (x[1] + u[0] + u[1])				\
  (x[1])					\
  /**/
#define OBSERVATION_MODEL			\
  (x[1])					\
  /**/
#define PROCESS_NOISE_COVARIANCE		\
  ( (0.1) (0) )					\
  ( (0) (0.1) )					\
  /**/
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (10) )					\
  /**/

// optional
//#define INITIAL_STATE
//#define INITIAL_ERROR_COVARIANCE
