/**
 * @file 
 * @author Denise Ratasich
 * @date 16.01.2013
 *
 * @brief Example configuration KF for navigation.
 *
 * This configuration only uses the wheel velocities to estimate the
 * position. Angular velocity and velocity into heading direction can
 * be calculated directly out of the wheel velocities.
 *
 * To improve the model the control inputs are used too. The velocity
 * should correspond to the topic cmd_vel, i.e. \f$v = \f$ \c
 * cmd_vel.linear.x and \f$\omega = \f$ \c cmd_vel.angular.z .
 */

// -----------------------------------------
// application specific defines and includes
// -----------------------------------------

#define ESTIMATION_PERIOD	100

#define FILTER_PERIOD	(0.1)		// filter period T

#define	RADIUS		(0.3)		// distance between wheels / 2 (in meters)

// -----------------------------------------
// input
// -----------------------------------------

// Get acceleration in heading direction and angular velocity around
// z-axis (rotation).
#define TOPICS_IN					\
  ((velocity_wheel_left) (std_msgs::Float32) (data))	\
  ((velocity_wheel_right) (std_msgs::Float32) (data))	\
  /**/

#define TOPICS_IN_CTRL					\
  ((cmd_vel) (geometry_msgs::Twist) (linear.x))		\
  ((cmd_vel) (geometry_msgs::Twist) (angular.z))	\
  /**/

// The message includes.
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// -----------------------------------------
// output
// -----------------------------------------

// Publish (nearly) pose (ds, heading).
#define TOPICS_OUT			\
  ((th) (0))				\
  ((omega) (1))				\
  ((ds) (2))				\
  ((v) (3))				\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// Use KF for estimation.
#define METHOD	KALMAN_FILTER

// Model.
// 
// x0: th = th + omega * T
// x1: omega = cmd_vel.angular.z
// x2: ds = v * T
// x3: v = cmd_vel.linear.x
// 
// u0: cmd_vel.linear.x
// u1: cmd_vel.angular.z
// 
// The wheel velocities are published in standard format of rosaria,
// i.e. mm/s. Hence these values have to be converted to m/s.
// 
// z0: velocity_wheel_left.data = (v - omega*R)*1000
// z0: velocity_wheel_right.data = (v + omega*R)*1000

// State transition model.
//
//    x0 x1 x2 x3
// x0 1  T  0  0
// x1 0  0  0  0
// x2 0  0  0  T
// x3 0  0  0  0
// 
#define STATE_TRANSITION_MODEL			\
  ( (1) (FILTER_PERIOD) (0) (0)		    )	\
  ( (0) (0)		(0) (0)		    )	\
  ( (0) (0)		(0) (FILTER_PERIOD) )	\
  ( (0) (0)		(0) (0)		    )	\
  /**/

#define CONTROL_INPUT_MODEL			\
  ( (0) (0) )					\
  ( (0) (1) )					\
  ( (0) (0) )					\
  ( (1) (0) )					\
  /**/

// Observation model.
// 
//    x0 x1      x2 x3
// z0 0  -1000*R 0  1000
// z1 0  1000*R  0  1000
// 
#define OBSERVATION_MODEL			\
  ( (0) (-RADIUS*1000) (0) (1000) )		\
  ( (0) (RADIUS*1000)  (0) (1000) )		\
  /**/

// Process noise covariance.
// 
// guess...
// max velocity = 0.7m/s
// max slope (rosaria) = 0.03m/100ms
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0) (0)   (0) (0)   )			\
  ( (0) (0.5) (0) (0)   )			\
  ( (0) (0)   (0) (0)   )			\
  ( (0) (0)   (0) (0.3) )			\
  /**/

// Measurement noise covariance.
// 
// The variance of the sensor, i.e. our measurement.
// 
// deviation (according test runs): 2mm/s
//
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (2) (0) )					\
  ( (0) (2) )					\
  /**/

// optional
// Initialize with start position (0).
#define INITIAL_STATE				\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  /**/
