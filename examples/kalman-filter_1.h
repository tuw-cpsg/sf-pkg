/**
 * @file 
 * @author Denise Ratasich
 * @date 03.09.2013
 *
 * @brief Example configuration KF 1.
 */

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
#define TOPICS_IN				\
  ((signal) (std_msgs::Float64) (data))		\
  /**/

#define TOPICS_IN_CTRL				\
  ((ctrl) (std_msgs::Float64) (data))		\
  /**/

// Define the includes needed for these topics!
#include <std_msgs/Float64.h>

// -----------------------------------------
// output
// -----------------------------------------

#define TOPICS_OUT				\
  ((state_0_fused) (0))				\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

#define METHOD				KALMAN_FILTER

// required
#define STATE_TRANSITION_MODEL		( (1)   )
#define PROCESS_NOISE_COVARIANCE	( (0.1) )
#define OBSERVATION_MODEL		( (1)   )
#define MEASUREMENT_NOISE_COVARIANCE	( (10)  )

// optional
#define CONTROL_INPUT_MODEL		( (1) )
//#define INITIAL_STATE			(0)
//#define INITIAL_ERROR_COVARIANCE	( (1) )

