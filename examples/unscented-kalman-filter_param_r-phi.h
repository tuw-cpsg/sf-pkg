/**
 * @file 
 * @author Denise Ratasich
 * @date 21.11.2013
 *
 * @brief Example configuration UKF for parameter estimation of
 * accelerometer KXTF9.
 *
 * In this example the radius and angle of the accelerometer
 * w.r.t. the base_link will be estimated. 
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
// general
// -----------------------------------------

#define ESTIMATION_PERIOD	100	// in ms!

// -----------------------------------------
// input
// -----------------------------------------

// Get acceleration in x- and y-axis (heading direction) and angular
// velocity around z-axis (rotation).
#define TOPICS_IN						\
  ((angular_velocity, vector.z, geometry_msgs::Vector3Stamped))	\
  ((acceleration, vector.x, geometry_msgs::Vector3Stamped))	\
  ((acceleration, vector.y, geometry_msgs::Vector3Stamped))	\
  /**/

// The message includes.
#include <geometry_msgs/Vector3Stamped.h>

// -----------------------------------------
// output
// -----------------------------------------

// Publish all state variables.
#define TOPICS_OUT				\
  ((omega, 0))					\
  ((ax, 1))					\
  ((ay, 2))					\
  ((phi, 3))					\
  ((r, 4))					\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// Use UKF for estimation.
#define METHOD	UNSCENTED_KALMAN_FILTER

// State transition model.
//
// Assumption: the robot is spinning, i.e. turning on a spot. So the
// acceleration in heading direction should be 0. Hence the radius and
// angle can be calculated directly out of ax and ay.
//
// x0: omega = omega
// x1: ax = ax			// x-direction of robot	
// x2: ay = ay			// y-direction of robot
// x3: phi = tan(ay / ax)
// x4: r = sqrt(ax^2 + ay^2)*T / |omega|
//
#define STATE_TRANSITION_MODEL					\
  ( x[0] )							\
  ( x[1] )							\
  ( x[2] )							\
  ( atan(x[2]/x[1]) )						\
  ( sqrt(x[1]*x[1] + x[2]*x[2])*FILTER_PERIOD / fabs(x[0]) )	\
  /**/
//  ( (x[1] < 0) ? (PI + atan(x[2]/x[1])) : (atan(x[2]/x[1])) )	\

// Observation model.
//
// All measurements arrive in fundamental units. But the accelerometer
// is not mounted according the robots coordinate frame, it is rotated
// by 90° w.r.t. the robots coordinate frame. Hence the robot's
// x-direction corresponds to the negative y-axis of the KXTF9. The
// y-direction of the robot is exactly the x-axis of the
// accelerometer.
//
// z0: IMU3000 angular_velocity.vector.z = omega
// z1: KXTF9 acceleration.vector.x = -ay
// z2: KXTF9 acceleration.vector.y = ax
//
#define OBSERVATION_MODEL			\
  (  x[0] )					\
  ( -x[2] )					\
  (  x[1] )					\
  /**/

// Process noise covariance.
// 
// There is no change in velocity or anything else, just spinning. The
// state should always remain the same (property of a parameter
// estimation).
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0) (0) (0) (0) (0) )			\
  ( (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) )			\
  ( (0) (0) (0) (0) (0) )			\
  /**/

// The variance of the sensors, i.e. our measurements.
// 
// KXTF9 worst case deviation: about 0.05g=0.5m/s^2 -> choose 0.5^2
// for variance
// IMU3000: about 0.01dps=0.00017rad/s
//
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (0.001)    (0)    (0) )			\
  ( (0)       (1)    (0) )			\
  ( (0)          (0) (1) )			\
  /**/

// optional

// Initialize with start position (1). Avoid x[2] (omega) and x[4]
// (ax) to be 0 -> zero-division!
#define INITIAL_STATE				\
  ( 0.1 )					\
  ( 0.001 )					\
  ( 0.001 )					\
  ( -0.6 )					\
  ( 0.006 )					\
  /**/
