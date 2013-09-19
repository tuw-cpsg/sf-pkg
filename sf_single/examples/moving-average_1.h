/**
 * @file 
 * @author Denise Ratasich
 * @date 03.09.2013
 *
 * @brief Example configuration MA 1.
 */

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

#define METHOD			MOVING_AVERAGE
#define WINDOW_SIZE		5
#define WEIGHTING_COEFFICIENTS	1, 2, 5, 2, 1
