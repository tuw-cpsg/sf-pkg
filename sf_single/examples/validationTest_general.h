/**
 * @file 
 * @author Denise Ratasich
 * @date 03.10.2013
 *
 * @brief Example configurations for testing the validation of UKF.
 */

// The signal's message include.
#include <std_msgs/Float64.h>

// --- valid ---

// #define METHOD	MOVING_MEDIAN
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))

// --- invalid ---

// method missing
#undef METHOD
#define TOPICS					\
  ((signal, data, std_msgs::Float64))		\
  ((signal, data, std_msgs::Float64))

// topics missing
// #define METHOD	MOVING_MEDIAN
