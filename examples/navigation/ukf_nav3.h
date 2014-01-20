/**
 * @file 
 * @author Denise Ratasich
 * @date 20.01.2014
 *
 * @brief Example configuration UKF for navigation.
 *
 * This configuration uses accelerometer, gyroscope and the wheel
 * velocities to estimate the position.
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

// Get the independent velocities of the wheels, acceleration and
// angular velocity of the gyroscope.
#define TOPICS_IN							\
  ((angular_velocity) (geometry_msgs::Vector3Stamped) (vector.z))	\
  ((acceleration) (geometry_msgs::Vector3Stamped) (vector.y))		\
  ((velocity_wheel_left) (std_msgs::Float32) (data))			\
  ((velocity_wheel_right) (std_msgs::Float32) (data))			\
  /**/

// The message includes.
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>

// -----------------------------------------
// output
// -----------------------------------------

// Publish all state variables.
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

// Use UKF for estimation.
#define METHOD	UNSCENTED_KALMAN_FILTER

// State transition model.
//
// Translation and rotation are assumed to be constant, i.e. we don't
// know what the robot will drive next.
//
// x0: x = x + ds * cos(th)
// x1: y = y + ds * sin(th)
// x2: th = th + omega * T
// x3: omega = omega
// x4: ds = v * T
// x5: v = v + a * T
// x6: a = a
//
#define STATE_TRANSITION_MODEL			\
  ( x[0] + x[4]*std::cos(x[2])	)		\
  ( x[1] + x[4]*std::sin(x[2])	)		\
  ( x[2] + x[3]*FILTER_T	)		\
  ( x[3]			)		\
  ( x[5]*FILTER_T		)		\
  ( x[5] + x[6]*FILTER_T	)		\
  ( x[6]			)		\
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
  ( x[3]		      )			\
  ( x[6]		      )			\
  ( (x[5] - x[3]*RADIUS)*1000 )			\
  ( (x[5] + x[3]*RADIUS)*1000 )			\
  /**/

// Process noise covariance.
// 
// guess...
// max velocity = 0.7m/s
// max slope (rosaria) = 0.03m/s per 100ms
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

// The variance of the sensor, i.e. our measurement.
// 
// KXTF9 worst case deviation: about 0.05g=0.5m/s/s -> choose 0.5^2
// for variance
// IMU3000: about 0.01dps = 0.00017rad/s
// WHEELS: deviation (according test runs): 2mm/s
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
  ( 0 )						\
  ( 0 )						\
  /**/
