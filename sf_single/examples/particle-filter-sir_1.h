/**
 * @file 
 * @author Denise Ratasich
 * @date 16.10.2013
 *
 * @brief Example configuration SIR (PF) 1.
 *
 * The system description consists of two state variables which
 * represent the same entity, but one has an offset which is realized
 * with the control input. The entity is measured directly, i.e. the
 * measurement represents the state variables directly.
 */

// -----------------------------------------
// input
// -----------------------------------------

// We only have one measurement or sensor to subscribe.
#define TOPICS_IN				\
  ((signal, data, std_msgs::Float64))		\
  /**/

// For the offset we subscribe to a specific control input.
#define TOPICS_IN_CTRL			\
  ((ctrl, data, std_msgs::Float64))	\
  /**/

// The message includes.
#include <std_msgs/Float64.h>

// -----------------------------------------
// output
// -----------------------------------------

// Publish both state variables.
#define TOPICS_OUT				\
  ((with_offset, 0))				\
  ((without_offset, 1))				\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

// Use the SIR (particle filter) for estimation.
#define METHOD	PARTICLE_FILTER_SIR

// Specify required parameters for the SIR (see documentation for
// which are needed).

// The state transition model is very simple. We expect the signal
// does not change (is a constant). Hence the old value is the new
// value. To the first signal we want to be able to add a "dynamic"
// offset, the control input.
// 
// x_apriori[0] = x[0] + u[0];
// x_apriori[1] = x[1];
//
#define STATE_TRANSITION_MODEL			\
  (x[0] + u[0])					\
  (x[1])					\
  /**/

// We assume the measurement is the state variable without offset,
// because x_1 represents our measured entity directly.
//
// z_expected[0] = x[1];
//
#define OBSERVATION_MODEL			\
  (x[1])					\
  /**/

// Say we trust the second state variable more than the first one with
// the offset.
#define PROCESS_NOISE_COVARIANCE \
  ( (0.5) (0)   )		 \
  ( (0)   (0.1) )		 \
  /**/

// The variance of the sensor, i.e. our measurement. We may know that
// our sensor has a standard deviation of 2, which gives a variance of
// 4.
#define MEASUREMENT_NOISE_COVARIANCE \
  ( (4) )			     \
  /**/

// The particle filter needs to be initialized with an initial state
// or state bounds. When the initial state is not known we can specify
// bounds for each state variable, i.e. the values the state variable
// could possibly take on. Because of a possible offset we give the
// first state variable a much bigger range.
#define STATE_BOUNDS				\
  ( (-20) (20) )				\
  ( (-10) (10) )				\
  /**/

// optional
#define NUMBER_OF_PARTICLES	10

