/**
 * @file 
 * @author Denise Ratasich
 * @date 06.09.2013
 *
 * @brief Example configuration EKF 1.
 */

// -----------------------------------------
// input
// -----------------------------------------

// Filter the same signal with different parameters. Define two
// inputs, so the EKF gets two measurements (that these measurements
// are the same the EKF doesn't know).
#define TOPICS					\
  ((signal, data, std_msgs::Float64))		\
  ((signal, data, std_msgs::Float64))		\
  /**/

// The signal's message include.
#include <std_msgs/Float64.h>

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// The estimator should be an EKF.
#define METHOD	EXTENDED_KALMAN_FILTER

// Specify required for the EKF (see documentation which are
// needed). The state transition model is very simple. We expect the
// signal does not change (is a constant). Hence the old value is the
// new value.
// 
// x_apriori[0] = x[0];
// x_apriori[1] = x[1];
//
#define STATE_TRANSITION_MODEL			\
  (x[0])					\
  (x[1])					\
  /**/

// The EKF needs a Jacobian for estimation (linearization of state
// transition model). Hence we must derive the above model to x[0] and
// x[1].
//
// df_0 / dx_0 = 1, df_0 / dx_1 = 0
// df_1 / dx_0 = 0, df_1 / dx_1 = 1
// with f_0 is the first formula in the model (x[0]) and f_1 is the
// second one.
//
#define STATE_TRANSITION_MODEL_JACOBIAN		\
  ( (1) (0) )					\
  ( (0) (1) )					\
  /**/

// We assume the measurements are the states, i.e. there is this
// sensor which measures our state variable directly. Because of that
// we assume the measurement to be exactly the previous state.
//
// z_expected[0] = x[0];
// z_expected[1] = x[1];
//
#define OBSERVATION_MODEL			\
  (x[0])					\
  (x[1])					\
  /**/

// As above we need the Jacobian of the model.
#define OBSERVATION_MODEL_JACOBIAN		\
  ( (1) (0) )					\
  ( (0) (1) )					\
  /**/

// The process noise covariance specifies how much we trust the system
// model, i.e. a higher value will cause the estimated state to follow
// more the measurements.
#define PROCESS_NOISE_COVARIANCE	{0.1,0} , {0,1}

// The measurement noise covariance specifies the variance of the
// sensors.
#define MEASUREMENT_NOISE_COVARIANCE	{10,0} , {0,10}

// optional
//#define CONTROL_INPUT
//#define INITIAL_STATE
//#define INITIAL_ERROR_COVARIANCE

