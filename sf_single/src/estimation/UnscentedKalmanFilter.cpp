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
    validated = false;
    // further initialization is done in validate()
  }

  UnscentedKalmanFilter::~UnscentedKalmanFilter () 
  {
    // no space to free
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  std::vector<double> UnscentedKalmanFilter::getState () const
  {
    std::vector<double> vx;
    
    // copy current state to std::vector
    for (int i = 0; i < x.size(); i++)
      vx.push_back(x[i]);

    return vx;
  }
  
  void UnscentedKalmanFilter::setInitialState (std::vector<double>& x0)
  {
    copy(x0, this->x);
    validated = false;
  }

  void UnscentedKalmanFilter::setInitialErrorCovariance(std::vector< std::vector<double> >& P0)
  {
    copy(P0, this->P);
    validated = false;
  }

  void UnscentedKalmanFilter::setStateTransitionModel (func_f f)
  {
    this->f = f;
    validated = false;
  }

  void UnscentedKalmanFilter::setControlInput (std::vector<double>& u) 
  {
    copy(u, this->u);
    validated = false;
  }

  void UnscentedKalmanFilter::setProcessNoiseCovariance (std::vector< std::vector<double> >& Q)
  {
    copy(Q, this->Q);
    validated = false;
  }

  void UnscentedKalmanFilter::setObservationModel (func_h h)
  {
    this->h = h;
    validated = false;
  }

  void UnscentedKalmanFilter::setMeasurementNoiseCovariance (std::vector< std::vector<double> >& R)
  {
    copy(R, this->R);
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

      // extract the values of next (the measurements) into a vector
      VectorXd z(R.rows());	// must have size m
      for (int i = 0; i < R.rows(); i++)
	z[i] = next[i].getValue();
    
      // Kalman Filtering ------------------------------------------
      // predict (time-update)
      // TODO augment process noise

      ut_function = STATE_TRANSITION_MODEL;
      UnscentedTransform ut1(x, P, this);
      ut1.compute();
      x = ut1.mean();
      P = ut1.covariance() + Q;
      
      // correct (measurement-update)
      // TODO augment measurement noise

      ut_function = OBSERVATION_MODEL;
      UnscentedTransform ut2(x, P, this);
      ut2.compute();
      VectorXd ze = ut2.mean();
      MatrixXd Pzz = ut2.covariance();
      MatrixXd Pxz = ut2.crossCovarianceXY();

      K = Pxz * (Pzz + R).inverse();
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

  Output UnscentedKalmanFilter::getLastEstimate(void) 
  {
    return out;
  }

  void UnscentedKalmanFilter::serialize(std::ostream& os) const
  {
    os << "UnscentedKalmanFilter" << std::endl
       << "state (x) = " << this->x << std::endl
       << "error covariance (P)" << std::endl << this->P << std::endl
       << "Kalman gain (K)" << std::endl << this->K << std::endl
       << "process noise covariance (Q)" << std::endl << this->Q << std::endl
       << "measurement noise covariance (R)" << std::endl << this->R << std::endl
       << "control input (u) = " << this->u << std::endl;
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

  // -----------------------------------------
  // private functions
  // -----------------------------------------
  void UnscentedKalmanFilter::copy(std::vector<double>& src, VectorXd& dest)
  {
    int rows = src.size();
    
    // specify size of destination matrix according src
    dest.resize(rows);		// needed, if dest-vector is empty

    // copy src to dest
    for (int row = 0; row < rows; row++)
      dest(row) = src[row];
  }

  void UnscentedKalmanFilter::copy(std::vector< std::vector<double> >& src, MatrixXd& dest)
  {
    // src: matrix represented by a collection of rows
    // matrix row: src[row]
    // matrix element: src[row][col]
    int rows = src.size();	// outer vector src = whole matrix
    int cols = src[0].size();	// inner vector represents one row

    // really a matrix? i.e. check if all rows have equal length
    for (int row = 0; row < rows; row++)
      if (src[row].size() != cols)
	throw std::length_error("Parameter not a matrix (different length of rows!).");

    // specify size of destination matrix according src
    dest.resize(rows,cols);	// needed, if dest-matrix is empty
    
    // copy src to dest
    for (int row = 0; row < rows; row++)
      for (int col = 0; col < cols; col++)
	dest(row,col) = src[row][col];
  }

  void UnscentedKalmanFilter::updateOutput(void)
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
