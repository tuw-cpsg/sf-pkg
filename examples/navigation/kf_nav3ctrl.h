/**
 * @file 
 * @author Denise Ratasich
 * @date 19.01.2013
 *
 * @brief Example configuration KF for navigation.
 *
 * This configuration uses the gyroscope (angular velocity), the
 * accelerometer (acceleration in heading direction) and the wheel
 * velocities to estimate the position.
 *
 * To improve the model the control inputs are used too. The
 * translation and rotation should correspond to the topic \c cmd_vel.
 */

// -----------------------------------------
// application specific defines and includes
// -----------------------------------------

#define ESTIMATION_PERIOD	100

// filter period T (must match ESTIMATION_PERIOD)
#define FILTER_T		(0.1)

// distance between wheels / 2 (in meters)
// guessed/estimated -> measure!
#define	RADIUS			(0.3)

// -----------------------------------------
// input
// -----------------------------------------

// Get the acceleration, angular velocity and the velocities of the
// left and right wheel.
#define TOPICS_IN							\
  ((angular_velocity) (geometry_msgs::Vector3Stamped) (vector.z))	\
  ((acceleration) (geometry_msgs::Vector3Stamped) (vector.y))		\
  ((velocity_wheel_left) (std_msgs::Float32) (data))			\
  ((velocity_wheel_right) (std_msgs::Float32) (data))			\
  /**/

#define TOPICS_IN_CTRL					\
  ((cmd_vel) (geometry_msgs::Twist) (linear.x))		\
  ((cmd_vel) (geometry_msgs::Twist) (angular.z))	\
  /**/

// The message includes.
#include <geometry_msgs/Vector3Stamped.h>
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
// x4: a = (cmd_vel.linear.x - v) / T
// 
// u0: cmd_vel.linear.x
// u1: cmd_vel.angular.z
// 
// The wheel velocities are published in standard format of rosaria,
// i.e. mm/s. Hence these values have to be converted to m/s.
// 
// z0: angular_velocity.z = omega
// z1: acceleration.y = a
// z2: velocity_wheel_left.data = (v - omega*R)*1000
// z3: velocity_wheel_right.data = (v + omega*R)*1000

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
  ( (0) (0)	   (0) (1)	     (FILTER_T) )	\
  ( (0) (0)	   (0) (-1/FILTER_T) (0)	)	\
  /**/

#define CONTROL_INPUT_MODEL			\
  ( (0) 	 (0) )				\
  ( (0) 	 (1) )				\
  ( (0) 	 (0) )				\
  ( (0)		 (0) )				\
  ( (1/FILTER_T) (0) )				\
  /**/

// Observation model.
// 
//    x0 x1      x2 x3   x4
// z0 0  1       0  0    0
// z1 0  0       0  0    1
// z0 0  -1000*R 0  1000 0
// z1 0  1000*R  0  1000 0
// 
#define OBSERVATION_MODEL			\
  ( (0) (1)	       (0) (0)	  (0) )		\
  ( (0) (0)	       (0) (0)	  (1) )		\
  ( (0) (-RADIUS*1000) (0) (1000) (0) )		\
  ( (0) (RADIUS*1000)  (0) (1000) (0) )		\
  /**/

// Process noise covariance.
// 
// guess...
// max velocity = 0.7m/s
// max slope (rosaria) = 0.03m/s/100ms
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0) (0)   (0) (0) (0)    )			\
  ( (0) (0.5) (0) (0) (0)    )			\
  ( (0) (0)   (0) (0) (0)    )			\
  ( (0) (0)   (0) (0) (0)    )			\
  ( (0) (0)   (0) (0) (0.3) )			\
  /**/

// Measurement noise covariance.
// 
// The variance of the sensor, i.e. our measurement.
// 
// KXTF9 worst case deviation: about 0.05g=0.5m/s^2 -> choose 0.5^2
// for variance
// IMU3000: about 0.01dps=0.00017rad/s
// WHEELS: deviation (according test runs) about 2mm/s
// 
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (0.00017) (0)    (0) (0) )			\
  ( (0)       (0.25) (0) (0) )			\
  ( (0)       (0)    (2) (0) )			\
  ( (0)       (0)    (0) (2) )			\
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
