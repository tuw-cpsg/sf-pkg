/**
 * @file 
 * @author Denise Ratasich
 * @date 02.10.2013
 *
 * @brief Examples for testing validation at compile time of KF.
 */

// Define the includes needed for these topics!
#include <std_msgs/Float64.h>

#define METHOD			KALMAN_FILTER

// --- valid ---

#define TOPICS_IN				\
  ((signal, data, std_msgs::Float64))
#define TOPICS_OUT				\
  ((signal_fused, 0))
#define STATE_TRANSITION_MODEL		((1))
#define OBSERVATION_MODEL		((1))
#define PROCESS_NOISE_COVARIANCE	((0.1))
#define MEASUREMENT_NOISE_COVARIANCE	((10))

// --- invalid ---

// // output topics missing
// #define TOPICS_IN				\
//   ((signal, data, std_msgs::Float64))
// #define STATE_TRANSITION_MODEL		((1))
// #define OBSERVATION_MODEL		((1))
// #define PROCESS_NOISE_COVARIANCE	((0.1))
// #define MEASUREMENT_NOISE_COVARIANCE	((10))

// #define TOPICS_IN					\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// // invalid size of STM
// #define STATE_TRANSITION_MODEL		((1)(0))
// // invalid size of OM, PNC, MNC
// #define STATE_TRANSITION_MODEL		((1)(0)) ((0)(1))
// #define OBSERVATION_MODEL		((1))
// #define PROCESS_NOISE_COVARIANCE	((0.1))
// #define MEASUREMENT_NOISE_COVARIANCE	((10))

// // #topics does not match OM
// #define TOPICS_IN					\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// #define STATE_TRANSITION_MODEL		((1))
// #define PROCESS_NOISE_COVARIANCE	((0.1))
// #define OBSERVATION_MODEL		((1))
// #define MEASUREMENT_NOISE_COVARIANCE	((10))

// // invalid size of MNC
// #define TOPICS_IN					\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// #define STATE_TRANSITION_MODEL		((1))
// #define PROCESS_NOISE_COVARIANCE	((0.1))
// #define OBSERVATION_MODEL		((0.5))((0.5))
// #define MEASUREMENT_NOISE_COVARIANCE	((10))

// // invalid size of PNC
// #define TOPICS_IN					\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// #define STATE_TRANSITION_MODEL		((1))
// #define PROCESS_NOISE_COVARIANCE	((0.1)(2))
// #define OBSERVATION_MODEL		((1))
// #define MEASUREMENT_NOISE_COVARIANCE	((10))

// // required params missing
// #define TOPICS_IN					\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// #define STATE_TRANSITION_MODEL		((1))
// #define PROCESS_NOISE_COVARIANCE	((0.1))
// #define OBSERVATION_MODEL		((1))

// // control input given, but CIM missing
// #define TOPICS_IN				\
//   ((signal, data, std_msgs::Float64)) 
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// #define STATE_TRANSITION_MODEL		((1))
// #define PROCESS_NOISE_COVARIANCE	((0.1))
// #define OBSERVATION_MODEL		((1))
// #define MEASUREMENT_NOISE_COVARIANCE	((10))
// #define TOPICS_IN_CTRL				\
//   ((ctrl, data, std_msgs::Float64))

// invalid size of control input model
// #define TOPICS_IN					\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// #define STATE_TRANSITION_MODEL		((1))
// #define PROCESS_NOISE_COVARIANCE	((0.1))
// #define OBSERVATION_MODEL		((1))
// #define MEASUREMENT_NOISE_COVARIANCE	((10))
// #define CONTROL_INPUT_MODEL		((1)(2))
// #define TOPICS_IN_CTRL				\
//   ((ctrl, data, std_msgs::Float64))

// // invalid size of x0, P0
// #define TOPICS_IN				\
//   ((signal, data, std_msgs::Float64))
// #define TOPICS_OUT				\
//   ((signal_fused, 0))
// #define STATE_TRANSITION_MODEL		((1))
// #define PROCESS_NOISE_COVARIANCE	((0.1))
// #define OBSERVATION_MODEL		((1))
// #define MEASUREMENT_NOISE_COVARIANCE	((10))
// #define INITIAL_STATE			(1)(2)
// #define INITIAL_ERROR_COVARIANCE	((1)(2))
