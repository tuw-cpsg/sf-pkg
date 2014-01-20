/**
 * @file 
 * @author Denise Ratasich
 * @date 20.01.2014
 *
 * @brief Example configuration EKF for navigation.
 *
 * This configuration only uses the accelerometer (KXTF9) and
 * gyroscope (IMU3000) to accumulate the current translation and
 * rotation to the pose.
 *
 * To improve the model the control inputs are used too. The
 * translation and rotation should correspond to the topic cmd_vel.
 */

// -----------------------------------------
// application specific defines and includes
// -----------------------------------------

// includes needed for the model (sin/cos)
#include <cmath>

#define ESTIMATION_PERIOD	(100)

// filter period T (must match ESTIMATION_PERIOD)
#define FILTER_T		(0.1)

// -----------------------------------------
// input
// -----------------------------------------

// Get acceleration in heading direction and angular velocity around
// z-axis.
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

// Use EKF for estimation.
#define METHOD	EXTENDED_KALMAN_FILTER

// State transition model.
// 
// x0: x = x + ds * cos(th)
// x1: y = y + ds * sin(th)
// x2: th = th + omega * T
// x3: omega = cmd_vel.angular.z
// x4: ds = v * T
// x5: v = v + a * T
// x6: a = (cmd_vel.linear.x - v) / T
// 
// u0: cmd_vel.linear.x
// u1: cmd_vel.angular.z
//
#define STATE_TRANSITION_MODEL			\
  ( x[0] + x[4]*std::cos(x[2]) )		\
  ( x[1] + x[4]*std::sin(x[2]) )		\
  ( x[2] + x[3]*FILTER_T       )		\
  ( u[1]		       )		\
  ( x[5]*FILTER_T	       )		\
  ( x[5] + x[6]*FILTER_T       )		\
  ( (u[0] - x[5]) / FILTER_T   )		\
  /**/

// Jacobian of STM to x.
// 
// df0/dx = (1) (0) (-ds*sin(th)) (0) (cos(th)) (0) (0)
// df1/dx = (0) (1) (ds*cos(th)) (0) (sin(th)) (0) (0)
// df2/dx = (0) (0) (1) (T) (0) (0) (0)
// df3/dx = (0) (0) (0) (1) (0) (0) (0)
// df4/dx = (0) (0) (0) (0) (0) (T) (0)
// df5/dx = (0) (0) (0) (0) (0) (1) (T)
// df5/dx = (0) (0) (0) (0) (0) (0) (1)
#define STATE_TRANSITION_MODEL_JACOBIAN					\
  ( (1) (0) (-x[4]*std::sin(x[2])) (0)	      (std::cos(x[2])) (0)	     (0)        ) \
  ( (0) (1) (x[4]*std::cos(x[2]))  (0)	      (std::sin(x[2])) (0)	     (0)        ) \
  ( (0) (0) (1)			   (FILTER_T) (0)	       (0)	     (0)        ) \
  ( (0) (0) (0)			   (0)	      (0)	       (0)	     (0)        ) \
  ( (0) (0) (0)			   (0)	      (0)	       (FILTER_T)    (0)        ) \
  ( (0) (0) (0)			   (0)	      (0)	       (1) 	     (FILTER_T) ) \
  ( (0) (0) (0)			   (0)	      (0)	       (-1/FILTER_T) (0)        ) \
  /**/

// Observation model.
//
// All measurements arrive in fundamental units. The acceleration in
// heading direction equals a_y of the KXTF9 because of the mounting
// position on the robot.
// 
// z0: angular_velocity.z = omega
// z1: acceleration.y = a
//
#define OBSERVATION_MODEL			\
  ( x[3] )					\
  ( x[6] )					\
  /**/

// Jacobian of OM to x.
//
// dh0/dx = (0) (0) (0) (1) (0) (0) (0)
// dh1/dx = (0) (0) (0) (0) (0) (0) (1)
#define OBSERVATION_MODEL_JACOBIAN		\
  ( (0) (0) (0) (1) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (1) )		\
  /**/

// Process noise covariance.
// 
// guess...
// max velocity = 0.7m/s
// max slope (rosaria) = 0.03m/100ms = 0.3m/s/s acceleration
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0) (0) (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0.5) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (0.3) )		\
  /**/

// Measurement noise covariance.
// 
// The variance of the sensor, i.e. our measurement.
// 
// KXTF9 worst case deviation: about 0.05g=0.5m/s/s -> choose 0.5^2
// for variance
// IMU3000: about 0.01dps = 0.00017rad/s
//
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (0.00017) (0)    )				\
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
  ( 0 )						\
  ( 0 )						\
  /**/
