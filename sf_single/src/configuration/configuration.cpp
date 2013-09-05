/**
 * @file 
 * @author Denise Ratasich
 * @date 05.09.2013
 *
 * @brief Configuration for ROS node.
 */

#include "configuration/configuration.h"
#include <string>

using namespace estimation;

// private functions
std::string getMethod();

void initEstimatorFactory(EstimatorFactory& factory)
{
  factory.addParam("method", getMethod());

#ifdef WINDOW_SIZE
  factory.addParam("window-size", WINDOW_SIZE);
#endif
#ifdef WEIGHTING_COEFFICIENTS
  EstimatorFactory::vector wc = { WEIGHTING_COEFFICIENTS };
  factory.addParam("weighting-coefficients", wc);
#endif

#ifdef STATE_TRANSITION_MODEL
  EstimatorFactory::matrix stm = { STATE_TRANSITION_MODEL };
  factory.addParam("state-transition-model", stm);
#endif
#ifdef PROCESS_NOISE_COVARIANCE
  EstimatorFactory::matrix pnc = { PROCESS_NOISE_COVARIANCE };
  factory.addParam("process-noise-covariance", pnc);
#endif
#ifdef OBSERVATION_MODEL
  EstimatorFactory::matrix om = { OBSERVATION_MODEL };
  factory.addParam("observation-model", om);
#endif
#ifdef MEASUREMENT_NOISE_COVARIANCE
  EstimatorFactory::matrix mnc = { MEASUREMENT_NOISE_COVARIANCE };
  factory.addParam("measurement-noise-covariance", mnc);
#endif
#ifdef CONTROL_INPUT_MODEL 
  EstimatorFactory::matrix cim = { CONTROL_INPUT_MODEL };
  factory.addParam("control-input-model", cim);
#endif
#ifdef CONTROL_INPUT
  EstimatorFactory::vector ci = { CONTROL_INPUT };
  factory.addParam("control-input", ci);
#endif
#ifdef INITIAL_STATE
  EstimatorFactory::vector is = { INITIAL_STATE };
  factory.addParam("initial-state", is);
#endif
#ifdef INITIAL_ERROR_COVARIANCE
  EstimatorFactory::matrix iec = { INITIAL_ERROR_COVARIANCE };
  factory.addParam("initial-error-covariance", iec);
#endif

  // Extended Kalman Filter: callbacks for STM and OM - "create" 2
  // callback functions! pass the function pointer to the factory
  // (boost::any can do function pointers)

  // callbacks for jacobian matrices A, H more difficult (variable
  // size) - do boost.preprocessor local iteration over all defines
  // MATRIX_i_j or tuples
}

std::string getMethod()
{
#if METHOD == MOVING_MEDIAN
  return "MovingMedian";
#elif METHOD == MOVING_AVERAGE
  return "MovingAverage";
#elif METHOD == KALMAN_FILTER
  return "KalmanFilter";
#elif METHOD == EXTENDED_KALMAN_FILTER
  return "ExtendedKalmanFilter";
#endif
}

#if METHOD == EXTENDED_KALMAN_FILTER
// 
// callback definitions come here
//
#endif
