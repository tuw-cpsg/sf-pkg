/**
 * @file 
 * @author Denise Ratasich
 * @date 19.09.2013
 *
 * @brief Example configuration header.
 *
 * Contains the defines which are used to create a special estimator.
 * According to the estimation method, chosen with the macro METHOD,
 * specific additional parameters have to be set. Strings have to be
 * set in quotes.
 *
 * Possible parameters for each method can be found in the
 * documentation of each method:
 * - \ref movingaverage
 * - \ref movingmedian
 * - \ref kalmanfilter
 * - \ref extendedkalmanfilter
 * - \ref unscentedkalmanfilter
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

// -----------------------------------------
// input
// -----------------------------------------

// List input topics here, format (no commas needed between topic
// tuples): ((name, field, type)).
#define TOPICS					\
  ((signal, data, std_msgs::Float64))		\
  /**/

// Define the includes needed for these topics!
#include <std_msgs/Float64.h>

// -----------------------------------------
// method and its parameters
// -----------------------------------------

#define METHOD	UNSCENTED_KALMAN_FILTER

// required
#define STATE_TRANSITION_MODEL			\
  (x[0])					\
  /**/
#define OBSERVATION_MODEL			\
  (x[0])					\
  /**/
#define PROCESS_NOISE_COVARIANCE	{0.1}
#define MEASUREMENT_NOISE_COVARIANCE	{10}

// optional
//#define CONTROL_INPUT
//#define INITIAL_STATE
//#define INITIAL_ERROR_COVARIANCE

#endif
