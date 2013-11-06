/**
 * @file 
 * @author Denise Ratasich
 * @date 05.11.2013
 *
 * @brief Example configuration SIR (PF) for navigation.
 *
 */

// -----------------------------------------
// application specific defines and includes
// -----------------------------------------

// #include <cmath>

#define FILTER_PERIOD	(0.1)		// filter period
#define GRAVITY		(9.81)		// 1g = 9m/s^2
#ifdef M_PI	
  #define PI	M_PI
#else
  #define PI	3.1415926536
#endif

// -----------------------------------------
// input
// -----------------------------------------

// Get x-acceleration and angular velocity around z-axis.
#define TOPICS_IN						\
  ((acceleration, vector.x, geometry_msgs::Vector3Stamped))	\
  ((angular_velocity, vector.z, geometry_msgs::Vector3Stamped))	\
  /**/

// The message includes.
#include <geometry_msgs/Vector3Stamped.h>

// -----------------------------------------
// output
// -----------------------------------------

// Publish pose (x, y and heading).
#define TOPICS_OUT			\
  ((x, 0))				\
  ((y, 1))				\
  ((theta, 2))				\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// Use SIR for estimation.
#define METHOD	PARTICLE_FILTER_SIR

// State transition model.
//
// x0: x = x + v_x * T
// x1: y = y + v_x * tan(omega * T)
// x2: th = th + omega * T
// x3: v_x = v_x + a_x * T
// x4: omega = omega
// x5: a_x = a_x (in m/s^2)
//
#define STATE_TRANSITION_MODEL			\
  (x[0] + x[3]*FILTER_PERIOD)			\
  (x[1] + x[3]*tan(x[4]*FILTER_PERIOD))		\
  (x[2] + x[4]*FILTER_PERIOD)			\
  (x[3] + x[5]*FILTER_PERIOD)			\
  (x[4])					\
  (x[5])					\
  /**/

// Observation model.
//
// z0: omega (in dps/s) = omega (in rad/s) * 180 / PI
// z1: a_x (in 1g) = a_x (in m/s^2) / G
//
#define OBSERVATION_MODEL			\
  (x[4] * 180 / PI)				\
  (x[5] / GRAVITY)				\
  /**/

// Process noise covariance.
// 
// guess...
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0.01) (0) (0) (0) (0) (0) )		\
  ( (0) (0.01) (0) (0) (0) (0) )		\
  ( (0) (0) (0.01) (0) (0) (0) )		\
  ( (0) (0) (0) (0.01) (0) (0) )		\
  ( (0) (0) (0) (0) (0.01) (0) )		\
  ( (0) (0) (0) (0) (0) (0.01) )		\
  /**/

// The variance of the sensor, i.e. our measurement.
// 
// KXTF9 worst case deviation: about 0.05g -> choose 0.03^2 for variance
// IMU3000: about 0.01dps
//
#define MEASUREMENT_NOISE_COVARIANCE \
  ( (0.0009)  (0) )		     \
  ( (0)    (0.01) )		     \
  /**/

// Initialize with start position (0).
#define INITIAL_STATE				\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  /**/

// optional
#define NUMBER_OF_PARTICLES	1000
