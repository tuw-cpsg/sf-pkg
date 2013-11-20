/**
 * @file 
 * @author Denise Ratasich
 * @date 02.10.2013
 *
 * @brief Validates the parameters for the ExtendedKalmanFilter.
 */

#ifndef __CONFIGURATION_VALIDATION_EXTENDEDKALMANFILTER_H__
#define __CONFIGURATION_VALIDATION_EXTENDEDKALMANFILTER_H__

// check for required parameters
#if !(defined STATE_TRANSITION_MODEL && \
      defined STATE_TRANSITION_MODEL_JACOBIAN && \
      defined PROCESS_NOISE_COVARIANCE && \
      defined OBSERVATION_MODEL && \
      defined OBSERVATION_MODEL_JACOBIAN && \
      defined MEASUREMENT_NOISE_COVARIANCE)
  #error "Validation of configuration header failed. ExtendedKalmanFilter: Missing required parameter(s)."
#else
// all required params given
// sizes
#define N VECTOR_SIZE(STATE_TRANSITION_MODEL)
#define M VECTOR_SIZE(OBSERVATION_MODEL)

// check sizes of input entities / topics
#if !(VECTOR_SIZE(TOPICS_IN) == M)
  #error "Validation of configuration header failed. ExtendedKalmanFilter: Number of inputs must match the observation model."
#endif

#ifdef TOPICS_IN_CTRL
  // the size of this vector must correspond to the usage in the state
  // transition model; errors with this input will cause runtime
  // exceptions!
#endif

// check sizes of required parameters
#if !(VECTOR_SIZE(STATE_TRANSITION_MODEL) == N)
  #error "Validation of configuration header failed. ExtendedKalmanFilter: State transition model has invalid size."
#endif
#if !(MATRIX_ROWS(STATE_TRANSITION_MODEL_JACOBIAN) == N  && \
      MATRIX_COLS(STATE_TRANSITION_MODEL_JACOBIAN) == N)
  #error "Validation of configuration header failed. ExtendedKalmanFilter: Jacobian of state transition model has invalid size."
#endif

#if !(MATRIX_ROWS(PROCESS_NOISE_COVARIANCE) == N  && \
      MATRIX_COLS(PROCESS_NOISE_COVARIANCE) == N)
  #error "Validation of configuration header failed. ExtendedKalmanFilter: Process noise covariance has invalid size."
#endif

#if !(VECTOR_SIZE(OBSERVATION_MODEL) == M)
  #error "Validation of configuration header failed. ExtendedKalmanFilter: Observation model has invalid size."
#endif
#if !(MATRIX_ROWS(OBSERVATION_MODEL_JACOBIAN) == M  && \
      MATRIX_COLS(OBSERVATION_MODEL_JACOBIAN) == N)
  #error "Validation of configuration header failed. ExtendedKalmanFilter: Jacobian of observation model has invalid size."
#endif

#if !(MATRIX_ROWS(MEASUREMENT_NOISE_COVARIANCE) == M  && \
      MATRIX_COLS(MEASUREMENT_NOISE_COVARIANCE) == M)
  #error "Validation of configuration header failed. ExtendedKalmanFilter: Measurement noise covariance has invalid size."
#endif

// check sizes of optional parameters
#ifdef INITIAL_STATE
  #if !(VECTOR_SIZE(INITIAL_STATE) == N)
    #error "Validation of configuration header failed. ExtendedKalmanFilter: Initial state has invalid size."
  #endif
#endif

#ifdef INITIAL_ERROR_COVARIANCE
  #if !(MATRIX_ROWS(INITIAL_ERROR_COVARIANCE) == N  && \
	MATRIX_COLS(INITIAL_ERROR_COVARIANCE) == N)
    #error "Validation of configuration header failed. ExtendedKalmanFilter: Initial error covariance has invalid size."
  #endif
#endif

#undef N
#undef M

#endif // check for required params

#endif // __CONFIGURATION_VALIDATION_EXTENDEDKALMANFILTER_H__
