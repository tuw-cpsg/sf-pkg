/**
 * @file 
 * @author Denise Ratasich
 * @date 16.10.2013
 *
 * @brief Example configuration SIR (PF) 1.
 *
 * The system description consists of one state variable which
 * represent a signal. The entity is measured directly, i.e. the
 * measurement represents the state variable directly.
 */

// -----------------------------------------
// general
// -----------------------------------------

// Estimate every 100ms.
#define ESTIMATION_PERIOD	100

// -----------------------------------------
// input
// -----------------------------------------

// We only have one signal to subscribe.
#define TOPICS_IN				\
  ((signal, data, std_msgs::Float64))		\
  /**/

// No control input.

// The message includes.
#include <std_msgs/Float64.h>

// -----------------------------------------
// output
// -----------------------------------------

// Publish both state variables.
#define TOPICS_OUT				\
  ((signal_estimated, 0))			\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// Use the SIR (particle filter) for estimation.
#define METHOD	PARTICLE_FILTER_SIR

// 
// Specify required parameters for the SIR (see documentation for
// which are needed).
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

// We assume the measurement is the state variable directly.
//
// z_expected[0] = x[0];
//
#define OBSERVATION_MODEL			\
  (x[0])					\
  /**/

// Guess.
#define PROCESS_NOISE_COVARIANCE		\
  ( (0.2) )					\
  /**/

// The variance of the signal. Guess.
#define MEASUREMENT_NOISE_COVARIANCE \
  ( (1) )			     \
  /**/

#define INITIAL_STATE			(0)

// 
// optional
// 

#define NUMBER_OF_PARTICLES		1000
#define INITIAL_ERROR_COVARIANCE	( (1) )
