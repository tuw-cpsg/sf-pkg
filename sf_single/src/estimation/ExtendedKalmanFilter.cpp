/**
 * @file 
 * @author Denise Ratasich
 * @date 05.09.2013
 *
 * @brief Implementation of an extended Kalman filter.
 */

#include "estimation/ExtendedKalmanFilter.h"

namespace estimation 
{
  ExtendedKalmanFilter::ExtendedKalmanFilter ()
  {
    validated = false;
    // further initialization is done in validate()
  }

  ExtendedKalmanFilter::~ExtendedKalmanFilter () 
  {
    // no space to free
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  std::vector<double> ExtendedKalmanFilter::getState () const
  {
    std::vector<double> vx;
    
    // copy current state to std::vector
    for (int i = 0; i < x.size(); i++)
      vx.push_back(x[i]);

    return vx;
  }
  
  void ExtendedKalmanFilter::setInitialState (std::vector<double>& x0)
  {
    copy(x0, this->x);
    validated = false;
  }

  void ExtendedKalmanFilter::setInitialErrorCovariance(std::vector< std::vector<double> >& P0)
  {
    copy(P0, this->P);
    validated = false;
  }

  void ExtendedKalmanFilter::setStateTransitionModel (func_f f)
  {
    this->f = f;
    validated = false;
  }

  void ExtendedKalmanFilter::setJacobianOfStateTransitionModel (func_df df)
  {
    this->df = df;
    validated = false;
  }

  void ExtendedKalmanFilter::setControlInput (std::vector<double>& u) 
  {
    copy(u, this->u);
    validated = false;
  }

  void ExtendedKalmanFilter::setProcessNoiseCovariance (std::vector< std::vector<double> >& Q)
  {
    copy(Q, this->Q);
    validated = false;
  }

  void ExtendedKalmanFilter::setObservationModel (func_h h)
  {
    this->h = h;
    validated = false;
  }

  void ExtendedKalmanFilter::setJacobianOfObservationModel (func_dh dh)
  {
    this->dh = dh;
    validated = false;
  }

  void ExtendedKalmanFilter::setMeasurementNoiseCovariance (std::vector< std::vector<double> >& R)
  {
    copy(R, this->R);
    validated = false;
  }

  void ExtendedKalmanFilter::validate (void)
  {
    try 
    {
      // check for minimal initialization, i.e. must not be empty:
      // state transition model and Jacobian, process noise
      // covariance, observation or measurement model and its
      // Jacobian, measurement noise covariance
      if (!f)			// callback empty
	throw std::runtime_error("State transition model missing.");
      if (!df)			// callback empty
	throw std::runtime_error("Jacobian of state transition model missing.");
      if (!h)			// callback empty
	throw std::runtime_error("Observation model missing.");
      if (!dh)			// callback empty
	throw std::runtime_error("Jacobian of observation model missing.");
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
	P = MatrixXd::Zero(n, n);
      // size of control input u cannot be checked in the EKF!
      // DANGEROUS! must be correctly initialized so that no out of
      // range error in the formulas of f/df occur!!

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

      if (A.rows() != n  ||  A.cols() != n)	// not initialized till now or invalid size from reinit
	A = MatrixXd::Zero(n, n);

      if (H.rows() != m  ||  H.cols() != n)	// not initialized till now or invalid size from reinit
	H = MatrixXd::Zero(m, n);

      // create output state
      if (out.size() != n) {		// not initialized till now or invalid size from reinit
	out.clear();
	for (int i = 0; i < n; i++)
	  out.add(OutputValue());
      }
    } 
    catch(std::exception& e) 
    {
      std::string additionalInfo = "ExtendedKalmanFilter: Validation failed. ";
      throw estimator_error(additionalInfo + e.what());
    }
  }

  // -----------------------------------------
  // IEstimator implementation
  // -----------------------------------------
  Output ExtendedKalmanFilter::estimate (Input next)
  {
    try 
    {
      if (!validated)
	throw std::runtime_error("Not yet validated!");

      // extract the values of next (the measurements) into a vector
      VectorXd z(H.rows());	// must have size m
      for (int i = 0; i < H.rows(); i++)
	z[i] = next[i].getValue();
    
      // Kalman Filtering ------------------------------------------
      // predict (time-update)
      // calculate A (the Jacobian of f at time step k-1)
      df(A,x,u);	
      // evaluate the a priori estimated state vector (x_{k-1} will be
      // overwritten to a priori x_k!)
      f(x,u);
      MatrixXd P_apriori = A*P*A.transpose() + Q;
    
      // update (measurement-update)
      // calc H, the Jacobian of h
      dh(H,x);
      K = (P_apriori*H.transpose()) * (H*P_apriori*H.transpose() + R).inverse();  
      VectorXd ze(H.rows());
      // calc estimated measurement vector -> ze
      h(ze,x);
      // calc a posteriori state vector -> x
      x = x + K*(z - ze);
      P = P_apriori - K*H*P_apriori;
      // -----------------------------------------------------------

      // insert new state and variance into the Output object
      updateOutput();
      return out;
    } 
    catch(std::exception& e) 
    {
      std::string additionalInfo = "ExtendedKalmanFilter: Estimation failed. ";
      throw estimator_error(additionalInfo + e.what());
    }
  }

  Output ExtendedKalmanFilter::getLastEstimate(void) 
  {
    return out;
  }

  void ExtendedKalmanFilter::serialize(std::ostream& os) const
  {
    os << "ExtendedKalmanFilter" << std::endl
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
  void ExtendedKalmanFilter::copy(std::vector<double>& src, VectorXd& dest)
  {
    int rows = src.size();
    
    // specify size of destination matrix according src
    dest.resize(rows);		// needed, if dest-vector is empty

    // copy src to dest
    for (int row = 0; row < rows; row++)
      dest(row) = src[row];
  }

  void ExtendedKalmanFilter::copy(std::vector< std::vector<double> >& src, MatrixXd& dest)
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

  void ExtendedKalmanFilter::updateOutput(void)
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
