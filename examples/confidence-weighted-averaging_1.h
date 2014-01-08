/**
 * @file 
 * @author Denise Ratasich
 * @date 02.01.2014
 *
 * @brief Example configuration CWA 1.
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
  ((signal_fused) (0))				\
  /**/

// -----------------------------------------
// method and its parameters
// -----------------------------------------

#define METHOD			CONFIDENCE_WEIGHTED_AVERAGING

#define IGNORE_ZERO_VARIANCE_VALUES
