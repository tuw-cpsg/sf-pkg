/**
 * @file 
 * @author Denise Ratasich
 * @date 27.01.2014
 *
 * @brief Example configuration EKF for navigation.
 *
 * This configuration uses accelerometer, gyroscope and the wheel
 * velocities to estimate the entities angular and linear velocity and
 * acceleration.
 *
 * Additionally a control input is used to enhance the model. \c
 * cmd_vel is used to give the desired translation and rotation.
 */

// -----------------------------------------
// application specific defines and includes
// -----------------------------------------

#include <cmath>

// period in which the estimation should be done
#define ESTIMATION_PERIOD	(100)		// in ms

// must match the ESTIMATION_PERIOD
#define FILTER_T		(0.1)		// filter period T in s

// distance between wheels / 2 (in meters), evaluated with parameter
// estimation using the gyroscope
#define	RADIUS			(0.3)

// -----------------------------------------
// input
// -----------------------------------------

// Get the independent velocities of the wheels.
#define TOPICS_IN							\
  ((angular_velocity) (geometry_msgs::Vector3Stamped) (vector.z))	\
  ((acceleration) (geometry_msgs::Vector3Stamped) (vector.y))		\
  ((velocity_wheel_left) (std_msgs::Float32) (data))			\
  ((velocity_wheel_right) (std_msgs::Float32) (data))			\
  /**/

// Get the driving commands for the robot, to know what the robot is
// doing.
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
  ((omega) (0))					\
  ((v) (1))					\
  ((a) (2))					\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// Use EKF for estimation.
#define METHOD	EXTENDED_KALMAN_FILTER

// State transition model.
//
// x0: omega = cmd_vel.angular.z
// x1: v = v + a * T
// x2: a = (cmd_vel.linear.x - v) / T
// 
// u0: cmd_vel.linear.x
// u1: cmd_vel.angular.z
//
#define STATE_TRANSITION_MODEL			\
  ( u[1]			)		\
  ( x[1] + x[2]*FILTER_T	)		\
  ( (u[0] - x[1]) / FILTER_T	)		\
  /**/

// Jacobian of STM to x.
// 
#define STATE_TRANSITION_MODEL_JACOBIAN		\
  ( (0)	       (0)	     (0) 	)	\
  ( (0)	       (1)	     (FILTER_T) )	\
  ( (0)	       (-1/FILTER_T) (0)	)	\
  /**/

// Observation model.
//
// The wheel velocities are published in standard format of rosaria,
// i.e. mm/s. Hence these values have to be converted to m/s.
//
// z0: angular_velocity.z = omega
// z1: acceleration.y = a
// z2: velocity_wheel_left.data = (v - omega*R)*1000
// z3: velocity_wheel_right.data = (v + omega*R)*1000
// 
#define OBSERVATION_MODEL			\
  ( x[0]		      )			\
  ( x[2]		      )			\
  ( (x[1] - x[0]*RADIUS)*1000 )			\
  ( (x[1] + x[0]*RADIUS)*1000 )			\
  /**/

// Jacobian of OM to x.
//
#define OBSERVATION_MODEL_JACOBIAN		\
  ((1)	          (0)	 (0) )			\
  ((0)	          (0)	 (1) )			\
  ((-RADIUS*1000) (1000) (0) )			\
  ((RADIUS*1000)  (1000) (0) )			\
  /**/

// Process noise covariance.
// 
// guess...
// max velocity = 0.7m/s
// max slope (rosaria) = 0.03m/s per 100ms
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0.5) (0) (0)   )				\
  ( (0)   (0) (0)   )				\
  ( (0)   (0) (0.3) )				\
  /**/

// The variance of the sensor, i.e. our measurement.
// 
// deviation (according test runs): 2mm/s
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
  /**/
