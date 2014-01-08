/**
 * @file 
 * @author Denise Ratasich
 * @date 03.09.2013
 *
 * @brief Example configuration MM 1.
 */

// -----------------------------------------
// input
// -----------------------------------------

#define TOPICS_IN				\
  ((signal) (std_msgs::Float64) (data))		\
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

#define METHOD			MOVING_MEDIAN
#define WINDOW_SIZE		5
