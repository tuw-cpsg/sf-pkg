/**
 * @file 
 * @author Denise Ratasich
 * @date 22.08.2013
 *
 * @brief Implementation of a linear Kalman filter.
 */

#include "estimation/KalmanFilter.h"
#include <stdexcept>
#include <iostream>

namespace estimation 
{
  KalmanFilter::KalmanFilter ()
  {
    // nothing to do
    // further initialization is done in validate()
  }

  KalmanFilter::KalmanFilter (std::vector< std::vector<double> > A,
			      std::vector< std::vector<double> > Q,
			      std::vector< std::vector<double> > H,
			      std::vector< std::vector<double> > R)
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
    // no space to free
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  std::vector<double> KalmanFilter::getState () const
  {
    std::vector<double> vx;
    
    // copy current state to std::vector
    for (int i = 0; i < x.size(); i++)
      vx.push_back(x[i]);

    return vx;
  }
  
  void KalmanFilter::setInitialState (std::vector<double>& x0)
  {
    copy(x0, this->x);
  }

  void KalmanFilter::setInitialErrorCovariance(std::vector< std::vector<double> >& P0)
  {
    copy(P0, this->P);
  }

  void KalmanFilter::setStateTransitionModel (std::vector< std::vector<double> >& A)
  {
    copy(A, this->A);
  }

  void KalmanFilter::setControlInputModel (std::vector< std::vector<double> >& B)
  {
    copy(B, this->B);
  }

  void KalmanFilter::setControlInput (std::vector<double>& u) 
  {
    copy(u, this->u);
  }

  void KalmanFilter::setProcessNoiseCovariance (std::vector< std::vector<double> >& Q)
  {
    copy(Q, this->Q);
  }

  void KalmanFilter::setObservationModel (std::vector< std::vector<double> >& H)
  {
    copy(H, this->H);
  }

  void KalmanFilter::setMeasurementNoiseCovariance (std::vector< std::vector<double> >& R)
  {
    copy(R, this->R);
  }

  void KalmanFilter::validate (void)
  {
    // check for minimal initialization, i.e. matrices must not be
    // empty: state transition model, process noise covariance,
    // observation or measurement model, measurement noise covariance
    if (A.rows() == 0) 	// empty matrix
      throw std::runtime_error("State estimation model missing.");
    if (H.rows() == 0) 	// empty matrix
      throw std::runtime_error("Observation model missing.");
    if (Q.rows() == 0) 	// empty matrix
      throw std::runtime_error("Process noise covariance missing.");
    if (R.rows() == 0) 	// empty matrix
      throw std::runtime_error("Measurement noise covariance missing.");
    
    // take sizes from required parameters
    int n = A.rows();
    int r = Q.rows();
    int m = R.rows();	// or H.rows() -> check below

    // create other parameters if missing ---------------------
    if (u.size() == 0)	// empty vector (control input missing)
      u = VectorXd::Zero(r);
    if (B.rows() == 0)	// empty matrix (control input model missing)
      B = MatrixXd::Zero(n, r);
    if (x.size() == 0)	// empty vector (initial state missing)
      x = VectorXd::Zero(n);
    if (P.rows() == 0)	// empty matrix (initial error covariance missing)
      P = MatrixXd::Zero(n, n);

    // check appropriate sizes of matrices and vectors --------
    if (x.rows() != n  ||  x.cols() != 1)
      throw std::runtime_error("Initial state has invalid size.");
    if (A.rows() != A.cols())
      throw std::runtime_error("State transition model must be a square matrix.");
    if (u.rows() != r  ||  u.cols() != 1)
      throw std::runtime_error("Control input has invalid size.");
    if (B.rows() != n  ||  B.cols() != r)
      throw std::runtime_error("Control input model has invalid size.");
    if (Q.rows() != Q.cols())
      throw std::runtime_error("Process noise covariance must be a square matrix.");
    if (H.rows() != m  ||  H.cols() != n)
      throw std::runtime_error("Observation model has invalid size.");
    if (R.rows() != R.cols())
      throw std::runtime_error("Measurement noise covariance must be a square matrix.");
    if (P.rows() != n  ||  P.cols() != n)
      throw std::runtime_error("Initial error covariance has invalid size.");

    // validation finished successfully -----------------------
    validated = true;

    // further things to initialize ---------------------------
    if (K.rows() == 0)		// not initialized till now
      K = MatrixXd::Zero(n, m);

    // create output state
    if (out.size() == 0)	// not initialized till now
      for (int i = 0; i < n; i++) {
	OutputValue val;
	out.add(val);
      }

    // debug
    std::stringstream ss;
    ss << "A: " << std::endl << A << std::endl << "---" << std::endl;
    ss << "B: " << std::endl << B << std::endl << "---" << std::endl;
    ss << "u: " << std::endl << u << std::endl << "---" << std::endl;
    ss << "Q: " << std::endl << Q << std::endl << "---" << std::endl;
    ss << "H: " << std::endl << H << std::endl << "---" << std::endl;
    ss << "R: " << std::endl << R << std::endl << "---" << std::endl;
    ss << "x: " << std::endl << x << std::endl << "---" << std::endl;
    ss << "P: " << std::endl << P << std::endl << "---" << std::endl;
    ss << "K: " << std::endl << K << std::endl << "---" << std::endl;
    std::cout << ss.str() << std::endl;
  }

  // -----------------------------------------
  // IEstimationMethod implementation
  // -----------------------------------------
  Output KalmanFilter::estimate (Input next)
  {
    if (!validated)
      return out;

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

  Output KalmanFilter::getLastEstimate(void) 
  {
    return out;
  }

  // -----------------------------------------
  // private functions
  // -----------------------------------------

  void KalmanFilter::copy(std::vector<double>& src, VectorXd& dest)
  {
    int rows = src.size();
    
    // specify size of destination matrix according src
    dest.resize(rows);		// needed, if dest-vector is empty

    // copy src to dest
    for (int row = 0; row < rows; row++)
      dest(row) = src[row];
  }

  void KalmanFilter::copy(std::vector< std::vector<double> >& src, MatrixXd& dest)
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

  void KalmanFilter::updateOutput(void)
  {
    // fill output with state and covariance
    for (int i = 0; i < x.size(); i++) {
      out[i].setValue(x[i]);
      out[i].setVariance(P(i,i));
      // TODO: fill jitter_ms
    }
  }
}
