/**
 * @file 
 * @author Denise Ratasich
 * @date 03.09.2013
 *
 * @brief Example configuration MA 2.
 */

// -----------------------------------------
// input
// -----------------------------------------

// List input topics here, format (no commas needed between topic
// tuples): ((name, field, type)).
#define TOPICS_IN				\
  ((signal, data, std_msgs::Float64))		\
  /**/

// Define the includes needed for these topics!
#include <std_msgs/Float64.h>

// -----------------------------------------
// output
// -----------------------------------------

#define TOPICS_OUT				\
  ((state_0_fused, 0))				\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

#define METHOD			MOVING_AVERAGE
#define WINDOW_SIZE		3
#define WEIGHTING_COEFFICIENTS			\
  (0.5) (0.2) (1) (3) (1)
