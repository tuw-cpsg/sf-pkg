/**
 * @file 
 * @author Denise Ratasich
 * @date 03.10.2013
 *
 * @brief Example configuration for testing validation of EKF.
 */

// The signal's message include.
#include <std_msgs/Float64.h>

#define METHOD	EXTENDED_KALMAN_FILTER

// --- valid ---

// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))

// #define STATE_TRANSITION_MODEL			\
//   (x[0])					\
//   (x[1])
// #define STATE_TRANSITION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1) )

// #define OBSERVATION_MODEL			\
//   (x[0])					\
//   (x[1])
// #define OBSERVATION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1) )

// #define PROCESS_NOISE_COVARIANCE \
//   ( (0.1) (0) )			 \
//   ( (0)   (1) )
// #define MEASUREMENT_NOISE_COVARIANCE \
//   ( (10)  (0) )			     \
//   ( (0)  (10) )


// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))

// #define STATE_TRANSITION_MODEL			\
//   (x[0])					\
//   (x[1])
// #define STATE_TRANSITION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1))
// #define OBSERVATION_MODEL			\
//   (x[0])
// #define OBSERVATION_MODEL_JACOBIAN		\
//   ( (1) (0) )

// #define PROCESS_NOISE_COVARIANCE \
//   ( (0.1) (0) )			 \
//   ( (0)   (1) )
// #define MEASUREMENT_NOISE_COVARIANCE		\
//   ( (10) )

// --- invalid ---

// // required params missing
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))

// #define STATE_TRANSITION_MODEL			\
//   (x[0])					\
//   (x[1])
// #define STATE_TRANSITION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1) )
// #define OBSERVATION_MODEL			\
//   (x[0])					\
//   (x[1])


// // invalid sizes
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))

// #define STATE_TRANSITION_MODEL			\
//   (x[0])
// #define STATE_TRANSITION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1) )
// #define OBSERVATION_MODEL			\
//   (x[0])					\
//   (x[1])
// #define OBSERVATION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1) )

// #define PROCESS_NOISE_COVARIANCE \
//   ( (0.1) (0) )			 \
//   ( (0)   (1) )
// #define MEASUREMENT_NOISE_COVARIANCE \
//   ( (10)  (0) )			     \
//   ( (0)  (10) )


// // invalid size of STM-jacobian
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))

// #define STATE_TRANSITION_MODEL			\
//   (x[0])					\
//   (x[1])
// #define STATE_TRANSITION_MODEL_JACOBIAN		\
//   ( (1) )					\
//   ( (0) )
// #define OBSERVATION_MODEL			\
//   (x[0])					\
//   (x[1])
// #define OBSERVATION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1) )

// #define PROCESS_NOISE_COVARIANCE \
//   ( (0.1) (0) )			 \
//   ( (0)   (1) )
// #define MEASUREMENT_NOISE_COVARIANCE \
//   ( (10)  (0) )			     \
//   ( (0)  (10) )


// // num topics invalid, invalid size of OM-jacobian and MNC
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))

// #define STATE_TRANSITION_MODEL			\
//   (x[0])					\
//   (x[1])
// #define STATE_TRANSITION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1))
// #define OBSERVATION_MODEL			\
//   (x[0])
// #define OBSERVATION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1) )

// #define PROCESS_NOISE_COVARIANCE \
//   ( (0.1) (0) )			 \
//   ( (0)   (1) )
// #define MEASUREMENT_NOISE_COVARIANCE \
//   ( (10)  (0) )			     \
//   ( (0)  (10) )


// // num topics invalid
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))		\
//   ((signal, data, std_msgs::Float64))

// #define STATE_TRANSITION_MODEL			\
//   (x[0])					\
//   (x[1])
// #define STATE_TRANSITION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1))
// #define OBSERVATION_MODEL			\
//   (x[0])
// #define OBSERVATION_MODEL_JACOBIAN		\
//   ( (1) (0) )

// #define PROCESS_NOISE_COVARIANCE \
//   ( (0.1) (0) )			 \
//   ( (0)   (1) )
// #define MEASUREMENT_NOISE_COVARIANCE		\
//   ( (10) )

// // invalid size of initial state
// #define TOPICS					\
//   ((signal, data, std_msgs::Float64))

// #define STATE_TRANSITION_MODEL			\
//   (x[0])					\
//   (x[1])
// #define STATE_TRANSITION_MODEL_JACOBIAN		\
//   ( (1) (0) )					\
//   ( (0) (1))
// #define OBSERVATION_MODEL			\
//   (x[0])
// #define OBSERVATION_MODEL_JACOBIAN		\
//   ( (1) (0) )
// #define PROCESS_NOISE_COVARIANCE \
//   ( (0.1) (0) )			 \
//   ( (0)   (1) )
// #define MEASUREMENT_NOISE_COVARIANCE		\
//   ( (10) )

// #define INITIAL_STATE	(1)

// invalid size of initial error covariance
#define TOPICS					\
  ((signal, data, std_msgs::Float64))

#define STATE_TRANSITION_MODEL			\
  (x[0])					\
  (x[1])
#define STATE_TRANSITION_MODEL_JACOBIAN		\
  ( (1) (0) )					\
  ( (0) (1))
#define OBSERVATION_MODEL			\
  (x[0])
#define OBSERVATION_MODEL_JACOBIAN		\
  ( (1) (0) )
#define PROCESS_NOISE_COVARIANCE \
  ( (0.1) (0) )			 \
  ( (0)   (1) )
#define MEASUREMENT_NOISE_COVARIANCE		\
  ( (10) )

#define INITIAL_STATE		(1)(2)
#define INITIAL_ERROR_COVARIANCE		\
  ( (0.1) )
