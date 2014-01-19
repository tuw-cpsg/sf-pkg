/**
 * @file 
 * @author Denise Ratasich
 * @date 19.01.2014
 *
 * @brief Example configuration EKF for estimating the distance
 * between the wheels of the robot.
 *
 * This configuration uses the gyroscope (angular velocity) and the
 * wheel velocities to estimate the distance between the wheels / 2
 * (radius needed for calculating the rotation of the robot).
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

// Get the angular velocity and the velocities of the left and right
// wheel.
#define TOPICS_IN							\
  ((angular_velocity) (geometry_msgs::Vector3Stamped) (vector.z))	\
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

// Publish all state variables.
#define TOPICS_OUT				\
  ((v) (0))					\
  ((omega) (1))					\
  ((r) (2))					\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// Use KF for parameter estimation. Its a nonlinear but simple model,
// so it should be suitable.
#define METHOD	EXTENDED_KALMAN_FILTER

// Model.
// 
// Rotation is assumed to be constant and the translation should be
// zero, i.e. the model is made for parameter estimation of the radius
// during spinning.
// 
// x0: v = cmd_vel.linear.x
// x1: omega = cmd_vel.angular.z
// x2: r = r
// 
// u0: cmd_vel.linear.x
// u1: cmd_vel.angular.z
//  
// The wheel velocities are published in standard format of rosaria,
// i.e. mm/s. Hence these values have to be converted to m/s.
// 
// z0: angular_velocity.z = omega
// z1: velocity_wheel_left.data = (v - omega * r) * 1000
// z2: velocity_wheel_right.data = (v + omega * r) * 1000

// State transition model.
//
#define STATE_TRANSITION_MODEL			\
  ( u[0] )					\
  ( u[1] )					\
  ( x[2] )					\
  /**/

// Jacobian of STM to x.
// 
// df0/dx = (0) (0) (0)
// df1/dx = (0) (0) (0)
// df3/dx = (0) (0) (1)
// 
#define STATE_TRANSITION_MODEL_JACOBIAN		\
  ( (0) (0) (0) )				\
  ( (0) (0) (0) )				\
  ( (0) (0) (1) )				\
  /**/

// Observation model.
//
#define OBSERVATION_MODEL			\
  ( x[1]		      )			\
  ( x[0] - x[1] * x[2] * 1000 )			\
  ( x[0] + x[1] * x[2] * 1000 )			\
  /**/

// Jacobian of OM to x.
// 
#define OBSERVATION_MODEL_JACOBIAN		\
  ( (0)	(1)	     (0)	  )		\
  ( (1) (-x[2]*1000) (-x[1]*1000) )		\
  ( (1) (x[2]*1000)  (x[1]*1000)  )		\
  /**/

// Process noise covariance.
//
// Everything should be constant during spinning.
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0.01) (0) (0) )				\
  ( (0) (0.01) (0) )				\
  ( (0) (0) (0.01) )				\
  /**/

// Measurement noise covariance.
// 
// The variance of the sensors, i.e. our measurements.
// 
// KXTF9 worst case deviation: about 0.05g=0.5m/s^2 -> choose 0.5^2
// for variance
// IMU3000: about 0.01dps=0.00017rad/s
// WHEELS: deviation (according test runs) about 2mm/s
// 
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (0.00017) (0) (0) )				\
  ( (0)       (2) (0) )				\
  ( (0)       (0) (2) )				\
  /**/

// optional
// Initialize with a guess if possible.
#define INITIAL_STATE				\
  ( 0 )						\
  ( 0 )						\
  ( 0.02 )					\
  /**/
