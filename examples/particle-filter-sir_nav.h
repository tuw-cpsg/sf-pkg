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

#include <cmath>

#define FILTER_PERIOD	(0.1)		// filter period
#define GRAVITY		(9.81)		// 1g = 9.81m/s^2
#ifdef M_PI	
  #define PI	M_PI
#else
#define PI	(3.1415926536)
#endif

// -----------------------------------------
// input
// -----------------------------------------

// Get x-acceleration and angular velocity around z-axis.
#define TOPICS_IN							\
  ((angular_velocity) (geometry_msgs::Vector3Stamped) (vector.z))	\
  ((acceleration) (geometry_msgs::Vector3Stamped) (vector.y))		\
  /**/

// The message includes.
#include <geometry_msgs/Vector3Stamped.h>

// -----------------------------------------
// output
// -----------------------------------------

// Publish pose (x, y and heading).
#define TOPICS_OUT			\
  ((x) (0))				\
  ((y) (1))				\
  ((th) (2))				\
  ((omega) (3))				\
  ((ds) (4))				\
  ((v) (5))				\
  ((a) (6))				\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// Use SIR for estimation.
#define METHOD	PARTICLE_FILTER_SIR

// State transition model.
//
// x0: x = x + ds * cos(th)
// x1: y = y + ds * sin(th)
// x2: th = th + omega * T
// x3: omega = omega (in rad/s)
// x4: ds = v * T
// x5: v = v + a * T
// x6: a = a (in m/s^2)
//
#define STATE_TRANSITION_MODEL			\
  (x[0] + x[4]*cos(x[2]))			\
  (x[1] + x[4]*sin(x[2]))			\
  (x[2] + x[3]*FILTER_PERIOD)			\
  (x[3])					\
  (x[5]*FILTER_PERIOD)				\
  (x[5] + x[6]*FILTER_PERIOD)			\
  (x[6])					\
  /**/

// Observation model.
//
// z0: omega_z (in dps/s) = omega (in rad/s) * 180 / PI
// z1: a_y (in 1g) = - a (in m/s^2) / G
//
#define OBSERVATION_MODEL			\
  (x[3] * 180 / PI)				\
  (- x[6] / GRAVITY)				\
  /**/

// Process noise covariance.
// 
// guess...
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0.0001) (0) (0) (0) (0) (0) (0) )		\
  ( (0) (0.0001) (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0.0001) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (1) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0.0001) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0.0001) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (1000) )		\
  /**/

// The variance of the sensor, i.e. our measurement.
// 
// KXTF9 worst case deviation: about 0.05g -> choose 0.03^2 for variance
// IMU3000: about 0.01dps
//
#define MEASUREMENT_NOISE_COVARIANCE \
  ( (0.01)      (0) )		     \
  ( (0)    (0.0009) )		     \
  /**/

// Initialize with start position (0).
#define INITIAL_STATE				\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  /**/

// optional
#define NUMBER_OF_PARTICLES	5000
