/**
 * @file 
 * @author Denise Ratasich
 * @date 22.08.2013
 *
 * @brief Implementation of a linear Kalman filter.
 */

#include "estimation/KalmanFilter.h"
#include <iostream>

namespace estimation 
{
  KalmanFilter::KalmanFilter ()
  {
    // everything needed is done in the constructor of
    // AbstractKalmanFilter
  }

  KalmanFilter::KalmanFilter (MatrixXd A,
			      MatrixXd Q,
			      MatrixXd H,
			      MatrixXd R)
  {
    // process
    setStateTransitionModel(A);
    setProcessNoiseCovariance(Q);

    // observation
    setObservationModel(H);
    setMeasurementNoiseCovariance(R);
  }

  KalmanFilter::~KalmanFilter () 
  {
    // nothing to do
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  void KalmanFilter::setStateTransitionModel (MatrixXd& A)
  {
    this->A = A;
    validated = false;
  }

  void KalmanFilter::setControlInputModel (MatrixXd& B)
  {
    this->B = B;
    validated = false;
  }

  void KalmanFilter::setObservationModel (MatrixXd& H)
  {
    this->H = H;
    validated = false;
  }

  void KalmanFilter::validate (void)
  {
    try 
    {
      // check for minimal initialization, i.e. matrices must not be
      // empty: state transition model, process noise covariance,
      // observation or measurement model, measurement noise covariance
      if (A.rows() == 0) 	// empty matrix
	throw std::runtime_error("State transition model missing.");
      if (H.rows() == 0) 	// empty matrix
	throw std::runtime_error("Observation model missing.");
      if (Q.rows() == 0) 	// empty matrix
	throw std::runtime_error("Process noise covariance missing.");
      if (R.rows() == 0) 	// empty matrix
	throw std::runtime_error("Measurement noise covariance missing.");
    
      // take sizes from required parameters
      int n = Q.rows(); // or A.rows() -> check below
      int m = R.rows();	// or H.rows() -> check below
      int l = 1;	// u/B are optional so set it to 1 -> smallest
			// possible matrix B

      // create other parameters if missing ---------------------
      if (B.rows() > 0) {
	// control input model given
	l = B.cols();
	if (u.size() != l) {	// empty vector or invalid size of u
	  u = VectorXd::Zero(l);
	} 
      } else if (u.size() > 0) {
	// no control model given - makes no sense..
	  throw std::runtime_error("Control input != 0, but no control input model given.");
      } else {
	// B and u must exist but should have no effect -> set to zero
	// (with smallest possible size: l = 1)
	B = MatrixXd::Zero(n, l);
	u = VectorXd::Zero(l);
      }

      if (x.size() != n)	// empty vector or invalid size from reinit
	x = VectorXd::Zero(n);
      if (P.rows() != n)	// empty matrix or invalid size from reinit
	P = MatrixXd::Zero(n, n);

      // check appropriate sizes of matrices and vectors --------
      if (A.rows() != n  ||  A.cols() != n)
	throw std::runtime_error("State transition model has invalid size.");
      if (H.rows() != m  ||  H.cols() != n)
	throw std::runtime_error("Observation model has invalid size.");
      if (Q.rows() != Q.cols())
	throw std::runtime_error("Process noise covariance must be a square matrix.");
      if (R.rows() != R.cols())
	throw std::runtime_error("Measurement noise covariance must be a square matrix.");
      if (u.rows() != l  ||  u.cols() != 1)
	throw std::runtime_error("Control input has invalid size.");
      if (B.rows() != n  ||  B.cols() != l)
	throw std::runtime_error("Control input model has invalid size.");
      if (x.rows() != n  ||  x.cols() != 1)
	throw std::runtime_error("Initial state has invalid size.");
      if (P.rows() != n  ||  P.cols() != n)
	throw std::runtime_error("Initial error covariance has invalid size.");

      // validation finished successfully -----------------------
      validated = true;

      // further things to initialize ---------------------------
      if (K.rows() != n  ||  K.cols() != m)	// not initialized till now or invalid size from reinit
	K = MatrixXd::Zero(n, m);

      // create output state
      if (out.size() != n) {			// not initialized till now or invalid size from reinit
	out.clear();
	for (int i = 0; i < n; i++)
	  out.add(OutputValue());
      }
    }
    catch(std::exception& e)
    {
      std::string additionalInfo = "KalmanFilter: Validation failed. ";
      throw estimator_error(additionalInfo + e.what());
    }
  }

  // -----------------------------------------
  // IEstimator implementation
  // -----------------------------------------
  Output KalmanFilter::estimate (Input next)
  {
    try 
    {
      if (!validated)
	throw std::runtime_error("Not yet validated.");

      // extract the values of next (the measurements) into a vector
      VectorXd z(H.rows());	// must have size m
      for (int i = 0; i < H.rows(); i++)
	z[i] = next[i].getValue();
    
      // Kalman Filtering ------------------------------------------
      // predict (time-update)
      VectorXd x_apriori = A*x + B*u;
      MatrixXd P_apriori = A*P*A.transpose() + Q;
    
      // update (measurement-update)
      K = (P_apriori*H.transpose()) * (H*P_apriori*H.transpose() + R).inverse();  
      x = x_apriori + K*(z - H*x_apriori);
      P = P_apriori - K*H*P_apriori;
      // -----------------------------------------------------------

      // insert new state and variance into the Output object
      updateOutput();
      return out;
    }
    catch(std::exception& e) 
    {
      std::string additionalInfo = "KalmanFilter: Estimation failed. ";
      throw estimator_error(additionalInfo + e.what());
    }
  }

  void KalmanFilter::serialize(std::ostream& os) const
  {
    os << "KalmanFilter" << std::endl
       << "state (x) = " << this->x << std::endl
       << "error covariance (P)" << std::endl << this->P << std::endl
       << "Kalman gain (K)" << std::endl << this->K << std::endl
       << "state transition model (A)" << std::endl << this->A << std::endl
       << "process noise covariance (Q)" << std::endl << this->Q << std::endl
       << "observation model (H)" << std::endl << this->H << std::endl
       << "measurement noise covariance (R)" << std::endl << this->R << std::endl
       << "control input (u) = " << this->u << std::endl
       << "control input model (B)" << std::endl << this->B << std::endl;
  }
}
