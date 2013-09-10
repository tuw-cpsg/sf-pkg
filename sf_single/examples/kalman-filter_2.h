/**
 * @file 
 * @author Denise Ratasich
 * @date 03.09.2013
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
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

// -----------------------------------------
// input
// -----------------------------------------

/** 
 * @brief List of input topics.
 * 
 * Define the inputs in a tuple: name, field to estimate, message
 * type, include for the message type. Do not place the name or field
 * within quotes!
 *
 * \note This macro represents a sequence of tuples, i.e. there is no
 * need for a comma, but has to be put into parentheses twice (inner
 * for the tuple, outer for the sequence).
 */
#define TOPICS					\
  ((signal, data, std_msgs::Float64))		\
  ((signal, data, std_msgs::Float64))		\
  /**/

// Define the includes needed for these topics!
#include <std_msgs/Float64.h>

// -----------------------------------------
// method and its parameters
// -----------------------------------------

#define METHOD				KALMAN_FILTER

// required
#define STATE_TRANSITION_MODEL		{1,0} , {0,1}
#define PROCESS_NOISE_COVARIANCE	{0.1,0} , {0,0.5}
#define OBSERVATION_MODEL		{1,0} , {0,1}
#define MEASUREMENT_NOISE_COVARIANCE	{10,0} , {0,10}

// optional
//#define CONTROL_INPUT_MODEL		{0}
//#define CONTROL_INPUT			0,0
#define INITIAL_STATE			0,0
#define INITIAL_ERROR_COVARIANCE	{1,0} , {0,1}

#endif
