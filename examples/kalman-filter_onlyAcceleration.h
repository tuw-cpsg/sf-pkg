/**
 * @file 
 * @author Denise Ratasich
 * @date 10.12.2013
 *
 * @brief Example configuration KF to filter constant acceleration.
 *
 * Filters the acceleration in x- and y-direction of the robot.
 */

// -----------------------------------------
// general
// -----------------------------------------

#define ESTIMATION_PERIOD	100

// -----------------------------------------
// input
// -----------------------------------------

// The noisy signal to estimate.
#define TOPICS_IN						\
  ((acceleration, vector.x, geometry_msgs::Vector3Stamped))	\
  ((acceleration, vector.y, geometry_msgs::Vector3Stamped))	\
  /**/

// no control input

// Define the includes needed for these topics!
#include <geometry_msgs/Vector3Stamped.h>

// -----------------------------------------
// output
// -----------------------------------------

// Filtered acceleration w.r.t. robot coordinate frame.
#define TOPICS_OUT				\
  ((ax, 0))					\
  ((ay, 1))					\
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
// x[0]: ax = ax
// x[1]: ay = ay
// 
#define STATE_TRANSITION_MODEL			\
  ( (1) (0) )					\
  ( (0) (1) )					\
  /**/

// Acceleration do not change when spinning.
#define PROCESS_NOISE_COVARIANCE		\
  ( (0.0001) (0) )				\
  ( (0) (0.0001) )				\
  /**/

// Convert between robot's and accelerometer's coordinate frame.
//
// acceleration.vector.x = ay
// acceleration.vector.y = -ax
//
#define OBSERVATION_MODEL			\
  (  (0) (1) )					\
  ( (-1) (0) )					\
  /**/

// Guess.
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (0.5) (0) )					\
  ( (0) (0.5) )					\
  /**/

//
// optional parameters
// 

//#define CONTROL_INPUT_MODEL		( (1) )

// Initialize state to 0.
#define INITIAL_STATE			(0.04)(0.03)

// Initialize variance to 1.
//#define INITIAL_ERROR_COVARIANCE	( (0.2) )

