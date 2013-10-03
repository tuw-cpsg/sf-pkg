/**
 * @file 
 * @author Denise Ratasich
 * @date 03.10.2013
 *
 * @brief Examples for testing validation at compile time of MM.
 */

#include <std_msgs/Float64.h>

#define METHOD			MOVING_MEDIAN

// --- valid ---

// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))

#define TOPICS					\
  ((signal, data, std_msgs::Float64))
#define WINDOW_SIZE	3

// --- invalid ---

// // only 1 topic allowed
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))

// // invalid window size
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))
// #define WINDOW_SIZE	-1

// // invalid window size
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))
// #define WINDOW_SIZE	0

// // invalid window size
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))
// #define WINDOW_SIZE	a
