/**
 * @file 
 * @author Denise Ratasich
 * @date 16.10.2013
 *
 * @brief Validates the parameters for the ParticleFilterSIR.
 */

#ifndef __CONFIGURATION_VALIDATION_PARTICLEFILTER_H__
#define __CONFIGURATION_VALIDATION_PARTICLEFILTER_H__

// check for required parameters
#if !(defined STATE_TRANSITION_MODEL && \
      defined PROCESS_NOISE_COVARIANCE && \
      defined OBSERVATION_MODEL && \
      defined MEASUREMENT_NOISE_COVARIANCE && \
      (defined INITIAL_STATE || defined STATE_BOUNDS))
  #error "Validation of configuration header failed. ParticleFilterSIR: Missing required parameter(s)."
#else
// all required params given
// sizes
#define N VECTOR_SIZE(STATE_TRANSITION_MODEL)
#define M VECTOR_SIZE(OBSERVATION_MODEL)

// check sizes of input entities / topics
#if !(VECTOR_SIZE(TOPICS_IN) == M)
  #error "Validation of configuration header failed. ParticleFilterSIR: Number of inputs must match the observation model."
#endif

#ifdef TOPICS_IN_CTRL
  // the size of this vector must correspond to the usage in the state
  // transition model; errors with this input parameter will cause
  // runtime exceptions!
#endif

// check sizes of required parameters
#if !(VECTOR_SIZE(STATE_TRANSITION_MODEL) == N)
  #error "Validation of configuration header failed. ParticleFilterSIR: State transition model has invalid size."
#endif

#if !(MATRIX_ROWS(PROCESS_NOISE_COVARIANCE) == N  && \
      MATRIX_COLS(PROCESS_NOISE_COVARIANCE) == N)
  #error "Validation of configuration header failed. ParticleFilterSIR: Process noise covariance has invalid size."
#endif

#if !(VECTOR_SIZE(OBSERVATION_MODEL) == M)
  #error "Validation of configuration header failed. ParticleFilterSIR: Observation model has invalid size."
#endif

#if !(MATRIX_ROWS(MEASUREMENT_NOISE_COVARIANCE) == M  && \
      MATRIX_COLS(MEASUREMENT_NOISE_COVARIANCE) == M)
  #error "Validation of configuration header failed. ParticleFilterSIR: Measurement noise covariance has invalid size."
#endif

#ifdef INITIAL_STATE
  #if !(VECTOR_SIZE(INITIAL_STATE) == N)
    #error "Validation of configuration header failed. ParticleFilterSIR: Initial state has invalid size."
  #endif
#endif

#ifdef STATE_BOUNDS
  #if !(MATRIX_ROWS(STATE_BOUNDS) == N  && \
        MATRIX_COLS(STATE_BOUNDS) == 2)
    #error "Validation of configuration header failed. ParticleFilterSIR: State bounds have invalid size."
  #endif
#endif

// check sizes of optional parameters
#ifdef NUMBER_OF_PARTICLES
  #if !(NUMBER_OF_PARTICLES > 0)
    #error "Validation of configuration header failed. ParticleFilterSIR: Initial error covariance has invalid size."
  #endif
#endif

#undef N
#undef M

#endif // check for required params

#endif // __CONFIGURATION_VALIDATION_UNSCENTEDKALMANFILTER_H__
