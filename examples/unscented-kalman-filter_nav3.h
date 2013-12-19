/**
 * @file 
 * @author Denise Ratasich
 * @date 16.12.2013
 *
 * @brief Example configuration UKF for navigation.
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

#define WHEEL_BASE_RADIUS	0.28	// m

// -----------------------------------------
// input
// -----------------------------------------

// Get acceleration in heading direction and angular velocity around
// z-axis (rotation).
#define TOPICS_IN						\
  ((angular_velocity, vector.z, geometry_msgs::Vector3Stamped))	\
  ((acceleration, vector.y, geometry_msgs::Vector3Stamped))	\
  ((velocity_wheel_left, data, std_msgs::Float32))	\
  ((velocity_wheel_right, data, std_msgs::Float32))	\
  /**/

// The message includes.
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>

// -----------------------------------------
// output
// -----------------------------------------

// Publish pose (x, y and heading).
#define TOPICS_OUT			\
  ((x, 0))				\
  ((y, 1))				\
  ((th, 2))				\
  ((omega, 3))				\
  ((ds, 4))				\
  ((v, 5))				\
  ((a, 6))				\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// Use UKF for estimation.
#define METHOD	UNSCENTED_KALMAN_FILTER

// State transition model.
//
// x0: x = x + ds * cos(th)
// x1: y = y + ds * sin(th)
// x2: th = th + omega * T
// x3: omega = omega
// x4: ds = v * T
// x5: v = v + a * T
// x6: a = a		// acceleration in heading direction
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
// All measurements arrive in fundamental units. The acceleration in
// heading direction equals a_y of the KXTF9 because of the mounting
// position on the robot.
//
// z0: omega_z = omega
// z1: a_y = a
// z2: v_l = v + omega*r
// z3: v_r = v - omega*r
//
#define OBSERVATION_MODEL			\
  (x[3])					\
  (x[6])					\
  (x[5] + x[3]*WHEEL_BASE_RADIUS)		\
  (x[5] - x[3]*WHEEL_BASE_RADIUS)		\
  /**/

// Process noise covariance.
// 
// We assume our formulas are the "perfect" model, only the formulas
// containing the acceleration and angular velocity of the sensor may
// vary. In the state transition model we assumed these variables do
// not change, but when accelerating the robot these values change of
// course. The following values in the process noise covariance
// specify the amount they can possibly change within a filter period
// (this is more a guess).
// 
// KXTF9: acceleration may change 10m/s^2 ... a guess
// IMU3000: max angular velocity is ??dps regarding the datasheet of
// the robot, so maybe 0.5rad/s is a good choice
//
#define PROCESS_NOISE_COVARIANCE		\
  ( (0) (0) (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0.5) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (0) )		\
  ( (0) (0) (0) (0) (0) (0.05) (0) )		\
  ( (0) (0) (0) (0) (0) (0) (0.5) )		\
  /**/

// The variance of the sensor, i.e. our measurement.
// 
// KXTF9 worst case deviation: about 0.05g=0.5m/s^2 -> choose 0.5^2
// for variance
// IMU3000: about 0.01dps=0.00017rad/s
// wheel encoders: about 2mm/s
//
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (0.00017)    (0) (0)     (0) )		\
  ( (0)       (0.25) (0)     (0) )		\
  ( (0)          (0) (0.002) (0) )		\
  ( (0)          (0) (0) (0.002) )		\
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
