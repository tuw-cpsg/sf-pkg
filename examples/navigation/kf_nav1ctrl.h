/**
 * @file 
 * @author Denise Ratasich
 * @date 19.01.2014
 *
 * @brief Example configuration KF for navigation.
 *
 * This configuration only uses the accelerometer (KXTF9) and
 * gyroscope (IMU3000) to accumulate the current translation and
 * rotation to the pose.
 *
 * To improve the model the control inputs are used too. The velocity
 * should correspond to the topic cmd_vel, i.e. \f$v = \f$ \c
 * cmd_vel.linear.x and \f$\omega = \f$ \c cmd_vel.angular.z .
 */

// -----------------------------------------
// application specific defines and includes
// -----------------------------------------

#define ESTIMATION_PERIOD	(100)

// filter period T (must match ESTIMATION_PERIOD)
#define FILTER_T		(0.1)		

// -----------------------------------------
// input
// -----------------------------------------

// Get acceleration in heading direction and angular velocity around
// z-axis (rotation).
#define TOPICS_IN							\
  ((angular_velocity) (geometry_msgs::Vector3Stamped) (vector.z))	\
  ((acceleration) (geometry_msgs::Vector3Stamped) (vector.y))		\
  /**/

#define TOPICS_IN_CTRL					\
  ((cmd_vel) (geometry_msgs::Twist) (linear.x))		\
  ((cmd_vel) (geometry_msgs::Twist) (angular.z))	\
  /**/

// The message includes.
#include <geometry_msgs/Vector3Stamped.h>
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
  ((a) (4))				\
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
// x3: v = v + a * T
// x4: a = (cmd_vel.linear.x - v) / T = cmd_vel.linear.x/T - v/T
//
// u0: cmd_vel.linear.x
// u1: cmd_vel.angular.z
// 
// All measurements arrive in fundamental units. The acceleration in
// heading direction equals a_y of the KXTF9 because of the mounting
// position on the robot.
// 
// z0: angular_velocity.z = omega
// z1: acceleration.y = a

// State transition model.
//
//    x0 x1 x2 x3    x4  
// x0 1  T  0  0     0   
// x1 0  0  0  0     0   
// x2 0  0  0  T     0   
// x3 0  0  0  1     T   
// x4 0  0  0  -1/T  0
//
#define STATE_TRANSITION_MODEL				\
  ( (1) (FILTER_T) (0) (0)	     (0)	)	\
  ( (0) (0)	   (0) (0)	     (0)	)	\
  ( (0) (0)	   (0) (FILTER_T)    (0)	)	\
  ( (0) (0)	   (0) (1) 	     (FILTER_T) )	\
  ( (0) (0)	   (0) (-1/FILTER_T) (0)	)	\
  /**/

#define CONTROL_INPUT_MODEL			\
  ( (0)		 (0) )				\
  ( (0)		 (1) )				\
  ( (0)		 (0) )				\
  ( (0)		 (0) )				\
  ( (1/FILTER_T) (0) )				\
  /**/

// Observation model.
// 
//    x0 x1 x2 x3 x4
// z0 0  1  0  0  0
// z1 0  0  0  0  1
// 
#define OBSERVATION_MODEL			\
  ( (0) (1) (0) (0) (0) )			\
  ( (0) (0) (0) (0) (1) )			\
  /**/

// Process noise covariance.
// 
// guess...
// cmd_vel may change from 0 to 0.7 maximal (max velocity = 0.7m/s)
//
// max slope (rosaria with predefined acceleration values) =
// 0.03m/100ms -> velocity 0.3m/s -> acceleration 3m/s/s
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0) (0)   (0) (0) (0)   )			\
  ( (0) (0.5) (0) (0) (0)   )			\
  ( (0) (0)   (0) (0) (0)   )			\
  ( (0) (0)   (0) (0) (0)   )			\
  ( (0) (0)   (0) (0) (0.3) )			\
  /**/

// Measurement noise covariance.
// 
// The variance of the sensors, i.e. our measurements.
// 
// KXTF9 worst case deviation: about 0.05g=0.5m/s^2 -> choose 0.5^2
// for variance
// IMU3000: about 0.01dps=0.00017rad/s
// 
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (0.00017)    (0) )				\
  ( (0)       (0.25) )				\
  /**/

// optional
// Initialize with start position (0).
#define INITIAL_STATE				\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  ( 0 )						\
  /**/
