/**
 * @file 
 * @author Denise Ratasich
 * @date 02.10.2013
 *
 * @brief Basic implementation of a Kalman filter.
 */

#include "estimation/AbstractKalmanFilter.h"

namespace estimation 
{
  AbstractKalmanFilter::AbstractKalmanFilter ()
  {
    validated = false;
    // further initialization is done in validate()
  }

  AbstractKalmanFilter::~AbstractKalmanFilter () 
  {
    // no space to free
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  VectorXd AbstractKalmanFilter::getState () const
  {
    return x;
  }
  
  void AbstractKalmanFilter::setInitialState (VectorXd& x0)
  {
    this->x = x0;
    validated = false;
  }

  void AbstractKalmanFilter::setInitialErrorCovariance(MatrixXd& P0)
  {
    this->P = P0;
    validated = false;
  }

  void AbstractKalmanFilter::setControlInput (VectorXd& u) 
  {
    this->u = u;
    validated = false;
  }

  void AbstractKalmanFilter::setProcessNoiseCovariance (MatrixXd& Q)
  {
    this->Q = Q;
    validated = false;
  }

  void AbstractKalmanFilter::setMeasurementNoiseCovariance (MatrixXd& R)
  {
    this->R = R;
    validated = false;
  }

  // -----------------------------------------
  // IEstimator implementation (partly)
  // -----------------------------------------
  Output AbstractKalmanFilter::getLastEstimate(void) 
  {
    return out;
  }

  void AbstractKalmanFilter::serialize(std::ostream& os) const
  {
    os << "AbstractKalmanFilter" << std::endl
       << "state (x) = " << this->x << std::endl
       << "error covariance (P)" << std::endl << this->P << std::endl
       << "Kalman gain (K)" << std::endl << this->K << std::endl
       << "process noise covariance (Q)" << std::endl << this->Q << std::endl
       << "measurement noise covariance (R)" << std::endl << this->R << std::endl
       << "control input (u) = " << this->u << std::endl;
  }

  // -----------------------------------------
  // private functions
  // -----------------------------------------
  void AbstractKalmanFilter::updateOutput(void)
  {
    if (out.size() != x.size())
      throw std::length_error("Output vector size != state vector size.");

    // fill output with state and covariance
    for (int i = 0; i < out.size(); i++) {
      out[i].setValue(x[i]);
      out[i].setVariance(P(i,i));
      // TODO: fill jitter_ms
    }
  }
}
