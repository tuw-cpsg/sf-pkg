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

// for all valid examples below
#define TOPICS_IN					\
  ((signal, data, std_msgs::Float64))
#define TOPICS_OUT					\
  ((signal_fused, 0))

#define WINDOW_SIZE	3

// --- invalid ---

// // only 1 topic allowed
// #define TOPICS_IN				\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))

// // only 1 topic allowed
// #define TOPICS_IN				\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused0, 0))				\
//   ((signal_fused1, 1))

// // invalid window size
// #define TOPICS_IN					\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// #define WINDOW_SIZE	-1

// // invalid window size
// #define TOPICS_IN					\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// #define WINDOW_SIZE	0

// // invalid window size
// #define TOPICS_IN					\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// #define WINDOW_SIZE	a
