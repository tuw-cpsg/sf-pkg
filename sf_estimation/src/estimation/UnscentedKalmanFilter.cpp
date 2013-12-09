/**
 * @file 
 * @author Denise Ratasich
 * @date 17.09.2013
 *
 * @brief Implementation of an unscented Kalman filter.
 */

#include "estimation/UnscentedKalmanFilter.h"
#include "estimation/UnscentedTransform.h"

namespace estimation 
{
  UnscentedKalmanFilter::UnscentedKalmanFilter ()
  {
    // everything needed is done in the constructor of
    // AbstractKalmanFilter
  }

  UnscentedKalmanFilter::~UnscentedKalmanFilter () 
  {
    // nothing to do
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  void UnscentedKalmanFilter::setStateTransitionModel (func_f f)
  {
    this->f = f;
    validated = false;
  }

  void UnscentedKalmanFilter::setObservationModel (func_h h)
  {
    this->h = h;
    validated = false;
  }

  void UnscentedKalmanFilter::validate (void)
  {
    try 
    {
      // check for minimal initialization, i.e. must not be empty:
      // state transition model, process noise covariance, observation
      // or measurement model, measurement noise covariance
      if (!f)			// callback empty
	throw std::runtime_error("State transition model missing.");
      if (!h)			// callback empty
	throw std::runtime_error("Observation model missing.");
      if (Q.rows() == 0) 	// empty matrix
	throw std::runtime_error("Process noise covariance missing.");
      if (R.rows() == 0) 	// empty matrix
	throw std::runtime_error("Measurement noise covariance missing.");

      // take sizes from required parameters
      int n = Q.rows();
      int m = R.rows();

      // create other parameters if missing ---------------------
      if (x.rows() != n)	// empty matrix or invalid size from reinit
	x = VectorXd::Zero(n);
      if (P.rows() != n)	// empty matrix or invalid size from reinit
	P = MatrixXd::Identity(n, n);
      // size of control input u cannot be checked in the UKF!
      // DANGEROUS! must be correctly initialized so that no out of
      // range error in the formulas of f occur!!

      // check appropriate sizes of matrices and vectors --------
      if (Q.rows() != Q.cols())
	throw std::runtime_error("Process noise covariance must be a square matrix.");
      if (R.rows() != R.cols())
	throw std::runtime_error("Measurement noise covariance must be a square matrix.");
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
      if (out.size() != n) {		// not initialized till now or invalid size from reinit
	out.clear();
	for (int i = 0; i < n; i++)
	  out.add(OutputValue());
      }
    } 
    catch(std::exception& e) 
    {
      std::string additionalInfo = "UnscentedKalmanFilter: Validation failed. ";
      throw estimator_error(additionalInfo + e.what());
    }
  }

  // -----------------------------------------
  // IEstimator implementation
  // -----------------------------------------
  Output UnscentedKalmanFilter::estimate (Input next)
  {
    try 
    {
      if (!validated)
	throw std::runtime_error("Not yet validated!");

      // measurement vector z
      VectorXd z(R.rows());	// must have size m
    
      // Kalman Filtering ------------------------------------------
      // predict (time-update)
      ut_function = STATE_TRANSITION_MODEL;
      UnscentedTransform ut1(x, P, this);
      ut1.compute();
      x = ut1.mean();
      P = ut1.covariance() + Q;
      
      // correct (measurement-update)
      ut_function = OBSERVATION_MODEL;
      UnscentedTransform ut2(x, P, this);
      ut2.compute();
      VectorXd ze = ut2.mean();
      prepareMeasurements(z, next, ze);
      MatrixXd Pzz = ut2.covariance() + R;
      MatrixXd Pxz = ut2.crossCovarianceXY();

      K = Pxz * Pzz.inverse();
      x = x + K * (z - ze);
      P = P - K * Pzz * K.transpose();
      // -----------------------------------------------------------

      // insert new state and variance into the Output object
      updateOutput();
      return out;
    } 
    catch(std::exception& e) 
    {
      std::string additionalInfo = "UnscentedKalmanFilter: Estimation failed. ";
      throw estimator_error(additionalInfo + e.what());
    }
  }

  void UnscentedKalmanFilter::serialize(std::ostream& os) const
  {
    os << "Unscented Kalman Filter";
  }

  // -----------------------------------------
  // ITransformer implementation
  // -----------------------------------------
  VectorXd UnscentedKalmanFilter::transform (const VectorXd& x)
  {
    VectorXd help;

    switch(ut_function)
    {
    case STATE_TRANSITION_MODEL:
      // evaluates the next expected state (a priori estimated state)
      help = x;
      f(help, u);
      return help;
      break;
    case OBSERVATION_MODEL:
      // evaluates the expected measurement considering the state
      // (sigmaPoint)
      help = VectorXd::Zero(R.rows());
      h(help, x);
      return help;
      break;
    default:
      throw std::runtime_error("UT transformer failed. Unknown UT function.");
    }
  }
}
