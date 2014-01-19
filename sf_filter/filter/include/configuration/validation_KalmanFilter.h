/**
 * @file 
 * @author Denise Ratasich
 * @date 02.10.2013
 *
 * @brief Validates the parameters for the KalmanFilter.
 */

#ifndef __CONFIGURATION_VALIDATION_KALMANFILTER_H__
#define __CONFIGURATION_VALIDATION_KALMANFILTER_H__

// check for required parameters
#if !(defined STATE_TRANSITION_MODEL && \
      defined PROCESS_NOISE_COVARIANCE && \
      defined OBSERVATION_MODEL && \
      defined MEASUREMENT_NOISE_COVARIANCE)
  #error "Validation of configuration header failed. KalmanFilter: Missing required parameter(s)."

#else
// all required params given
// sizes
#define N MATRIX_ROWS(STATE_TRANSITION_MODEL)
#define M MATRIX_ROWS(OBSERVATION_MODEL)
#if defined TOPICS_IN_CTRL
  #define L VECTOR_SIZE(TOPICS_IN_CTRL)
#elif defined CONTROL_INPUT_MODEL
  #define L MATRIX_COLS(CONTROL_INPUT_MODEL)
#endif

// check sizes of input entities / topics
#if !(VECTOR_SIZE(TOPICS_IN) == M)
  #error "Validation of configuration header failed. KalmanFilter: Number of inputs must match the observation model."
#endif

#if defined TOPICS_IN_CTRL  &&  !(defined CONTROL_INPUT_MODEL)
  #error "Validation of configuration header failed. KalmanFilter: Control input given, but no control input model."
#endif

// check sizes of required parameters
#if !(MATRIX_ROWS(STATE_TRANSITION_MODEL) == N  && \
      MATRIX_COLS(STATE_TRANSITION_MODEL) == N)
  #error "Validation of configuration header failed. KalmanFilter: State transition model has invalid size."
#endif

#if !(MATRIX_ROWS(PROCESS_NOISE_COVARIANCE) == N  && \
      MATRIX_COLS(PROCESS_NOISE_COVARIANCE) == N)
  #error "Validation of configuration header failed. KalmanFilter: Process noise covariance has invalid size."
#endif

#if !(MATRIX_ROWS(OBSERVATION_MODEL) == M  && \
      MATRIX_COLS(OBSERVATION_MODEL) == N)
  #error "Validation of configuration header failed. KalmanFilter: Observation model has invalid size."
#endif

#if !(MATRIX_ROWS(MEASUREMENT_NOISE_COVARIANCE) == M  && \
      MATRIX_COLS(MEASUREMENT_NOISE_COVARIANCE) == M)
  #error "Validation of configuration header failed. KalmanFilter: Measurement noise covariance has invalid size."
#endif

// check sizes of optional parameters
#ifdef CONTROL_INPUT_MODEL
  #if !(MATRIX_ROWS(CONTROL_INPUT_MODEL) == N  && \
	MATRIX_COLS(CONTROL_INPUT_MODEL) == L)
    #error "Validation of configuration header failed. KalmanFilter: Control input model has invalid size."
  #endif
#endif

#ifdef INITIAL_STATE
  #if !(VECTOR_SIZE(INITIAL_STATE) == N)
    #error "Validation of configuration header failed. KalmanFilter: Initial state has invalid size."
  #endif
#endif

#ifdef INITIAL_ERROR_COVARIANCE
  #if !(MATRIX_ROWS(INITIAL_ERROR_COVARIANCE) == N  && \
	MATRIX_COLS(INITIAL_ERROR_COVARIANCE) == N)
    #error "Validation of configuration header failed. KalmanFilter: Initial error covariance has invalid size."
  #endif
#endif

#undef N
#undef M
#undef L

#endif // check for required params

#endif // __CONFIGURATION_VALIDATION_KALMANFILTER_H__
