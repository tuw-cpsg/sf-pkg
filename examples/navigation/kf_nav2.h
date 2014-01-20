/**
 * @file 
 * @author Denise Ratasich
 * @date 19.01.2014
 *
 * @brief Example configuration KF for navigation.
 *
 * This configuration only uses the wheel velocities to estimate the
 * position. Angular velocity and velocity into heading direction can
 * be calculated directly out of the wheel velocities.
 */

// -----------------------------------------
// application specific defines and includes
// -----------------------------------------

#define ESTIMATION_PERIOD	(100)

// filter period T (must match ESTIMATION_PERIOD)
#define FILTER_T		(0.1)

// distance between wheels / 2 (in meters)
// guessed/estimated -> measure!
#define	RADIUS			(0.3)		
		

// -----------------------------------------
// input
// -----------------------------------------

// Get the velocities of the left and right wheel.
#define TOPICS_IN							\
  ((velocity_wheel_left) (std_msgs::Float32) (data))			\
  ((velocity_wheel_right) (std_msgs::Float32) (data))			\
  /**/

// The message includes.
#include <std_msgs/Float32.h>

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
// Translation and rotation are assumed to be constant, i.e. we don't
// know what the robot will drive next.
// 
// x0: th = th + omega * T
// x1: omega = omega
// x2: ds = v * T
// x3: v = v
//  
// The wheel velocities are published in standard format of rosaria,
// i.e. mm/s. Hence these values have to be converted to m/s.
// 
// z0: velocity_wheel_left.data = (v - omega*R)*1000
// z1: velocity_wheel_right.data = (v + omega*R)*1000

// State transition model.
//
//    x0 x1 x2 x3
// x0 1  T  0  0 
// x1 0  1  0  0 
// x2 0  0  0  T 
// x3 0  0  0  1 
// 
#define STATE_TRANSITION_MODEL			\
  ( (1) (FILTER_T) (0) (0)	  )		\
  ( (0) (1)	   (0) (0)	  )		\
  ( (0) (0)	   (0) (FILTER_T) )		\
  ( (0) (0)	   (0) (1)	  )		\
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
// max slope (rosaria) = 0.03m/s/100ms = 0.3m/s/s
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0) (0)   (0) (0)   )			\
  ( (0) (0.5) (0) (0)   )			\
  ( (0) (0)   (0) (0)   )			\
  ( (0) (0)   (0) (0.3) )			\
  /**/

// Measurement noise covariance.
// 
// The variance of the sensors, i.e. our measurements.
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
