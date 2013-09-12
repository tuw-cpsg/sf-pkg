/**
 * @file 
 * @author Denise Ratasich
 * @date 05.09.2013
 *
 * @brief Configuration for ROS node.
 */

#include "configuration/configuration.h"
#include <Eigen/Core>
#include <string>
#include <stdexcept>

using namespace estimation;
using namespace Eigen;

// private functions
std::string getMethod();

#if METHOD == EXTENDED_KALMAN_FILTER
void f(VectorXd& x, const VectorXd& u);
void df(MatrixXd& A, const VectorXd& x, const VectorXd& u);
void h(VectorXd& z, const VectorXd& x);
void dh(MatrixXd& H, const VectorXd& x);
#endif

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
  #if METHOD == KALMAN_FILTER
  EstimatorFactory::matrix stm = { STATE_TRANSITION_MODEL };
  #elif METHOD == EXTENDED_KALMAN_FILTER
  ExtendedKalmanFilter::func_f stm = f;
  #endif
  factory.addParam("state-transition-model", stm);
#endif
#ifdef PROCESS_NOISE_COVARIANCE
  EstimatorFactory::matrix pnc = { PROCESS_NOISE_COVARIANCE };
  factory.addParam("process-noise-covariance", pnc);
#endif
#ifdef OBSERVATION_MODEL
  #if METHOD == KALMAN_FILTER
  EstimatorFactory::matrix om = { OBSERVATION_MODEL };
  #elif METHOD == EXTENDED_KALMAN_FILTER
  ExtendedKalmanFilter::func_h om = h;
  #endif
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

#ifdef STATE_SIZE
  factory.addParam("state-size", STATE_SIZE);
#endif
#ifdef STATE_TRANSITION_MODEL_JACOBIAN
  ExtendedKalmanFilter::func_df stmj = df;
  factory.addParam("state-transition-model-jacobian", stmj);
#endif
#ifdef OBSERVATION_MODEL_JACOBIAN
  ExtendedKalmanFilter::func_dh omj = dh;
  factory.addParam("observation-model-jacobian", omj);
#endif
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
void f(VectorXd& x, const VectorXd& u)
{
  if (x.size() != STATE_SIZE)
    throw std::runtime_error("Applying state transition model failed, state vector has invalid size.");

  // create state vector for result, i.e. the a priori state estimate
  VectorXd x_apriori(x.size());

  // assign formulas of the state transition model to vector x,
  // i.e. calculate a priori state estimate
  CODE_ASSIGN_FORMULAS_TO_VECTOR(x_apriori, STATE_TRANSITION_MODEL);

  // copy back, x represents now the a priori state estimate
  x = x_apriori;
}

void df(MatrixXd& A, const VectorXd& x, const VectorXd& u)
{
  CODE_ASSIGN_FORMULAS_TO_MATRIX(A, STATE_TRANSITION_MODEL_JACOBIAN);
}

void h(VectorXd& z, const VectorXd& x)
{
  CODE_ASSIGN_FORMULAS_TO_VECTOR(z, OBSERVATION_MODEL);
}

void dh(MatrixXd& H, const VectorXd& x)
{
  CODE_ASSIGN_FORMULAS_TO_MATRIX(H, OBSERVATION_MODEL_JACOBIAN);
}
#endif
