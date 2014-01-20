/**
 * @file 
 * @author Denise Ratasich
 * @date 09.12.2013
 *
 * @brief Example configuration EKF to filter a constant signal.
 *
 * Filters a single raw signal. It is assumed to be constant. Its the
 * simplest extended Kalman filter with a state and measurement vector
 * of size 1.
 */

// -----------------------------------------
// input
// -----------------------------------------

// The noisy signal to estimate.
#define TOPICS_IN				\
  ((signal) (std_msgs::Float64) (data))		\
  /**/

// No control input.

// The message includes.
#include <std_msgs/Float64.h>

// -----------------------------------------
// output
// -----------------------------------------

// Publish the estimated signal.
#define TOPICS_OUT				\
  ((signal_estimated) (0))			\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// The estimator should be an EKF.
#define METHOD	EXTENDED_KALMAN_FILTER

// 
// required
// 

// The state transition model is very simple. We expect the signal
// does not change (is a constant). Hence the old value is the new
// value.
// 
// x_apriori[0] = x[0];
//
#define STATE_TRANSITION_MODEL			\
  (x[0])					\
  /**/

// The EKF needs a Jacobian for estimation (linearization of state
// transition model). Hence we must derive the above model to x[0].
//
// df_0 / dx_0 = 1
// with: f_0 is the first formula in the model (x[0]).
//
#define STATE_TRANSITION_MODEL_JACOBIAN		\
  ( (1) )					\
  /**/

// We assume the measurement is the state, i.e. there is this
// sensor which measures our state variable directly. Because of that
// we assume the measurement to be exactly the previous state.
//
// z_expected[0] = x[0];
//
#define OBSERVATION_MODEL			\
  (x[0])					\
  /**/

// As above we need the Jacobian of the model.
#define OBSERVATION_MODEL_JACOBIAN		\
  ( (1) )					\
  /**/

// The process noise covariance specifies how much we trust the system
// model, i.e. a higher value will cause the estimated state to follow
// more the measurements.
#define PROCESS_NOISE_COVARIANCE \
  ( (0.2) )			 \
  /**/

// The measurement noise covariance specifies the variance of the
// sensors.
#define MEASUREMENT_NOISE_COVARIANCE \
  ( (1) )			     \
  /**/

// 
// optional
// 

// Initialize state to 0.
#define INITIAL_STATE			(0)

// Initialize variance to 1.
#define INITIAL_ERROR_COVARIANCE	( (1) )

