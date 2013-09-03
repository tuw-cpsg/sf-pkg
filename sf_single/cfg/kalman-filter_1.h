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

#define TOPIC_NAME	"signal"
#define TOPIC_TYPE	std_msgs::Float64
#define TOPIC_FIELD	data

// -----------------------------------------
// method and its parameters
// -----------------------------------------

#define METHOD				"KalmanFilter"

// required
#define STATE_TRANSITION_MODEL		{1}
#define PROCESS_NOISE_COVARIANCE	{0.1}
#define OBSERVATION_MODEL		{1}
#define MEASUREMENT_NOISE_COVARIANCE	{10}

// optional
#define CONTROL_INPUT_MODEL		{0}
#define CONTROL_INPUT			0
#define INITIAL_STATE			0
#define INITIAL_ERROR_COVARIANCE	{1}

#endif
