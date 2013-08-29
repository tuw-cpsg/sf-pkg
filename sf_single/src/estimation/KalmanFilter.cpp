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
    // set parameters to default values
    init(); 

    // create output state
    for (int i = 0; i < 1; i++) {
      OutputValue val;
      out.add(val);
    }
  }

  KalmanFilter::KalmanFilter (std::vector<double> x0,
			      std::vector< std::vector<double> > A,
			      std::vector< std::vector<double> > Q,
			      std::vector< std::vector<double> > H,
			      std::vector< std::vector<double> > R)
  {
    // process
    int n = x0.size();
    setInitialState(x0);		// n
    setStateTransitionModel(A);		// n x n
    setProcessNoiseCovariance(Q);	// r x r
    int r = 1;				// r = 1 (because B, u not specified)
    B = MatrixXd::Zero(n, r);		// n x r
    u = VectorXd::Zero(r);		// r

    // observation
    int m = H.size();
    setObservationModel(H);		// m x n (the measurements z have size m)
    setMeasurementNoiseCovariance(R);	// m x m

    // intern
    P = MatrixXd::Zero(x.size(), x.size());	// n x n
    K = MatrixXd::Zero(x.size(), x.size());	// n x m
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
    // debug
    std::stringstream ss;
    ss << "A: " << std::endl << A << std::endl << "---" << std::endl;
    ss << "x: " << std::endl << x << std::endl << "---" << std::endl;
    ss << "B: " << std::endl << B << std::endl << "---" << std::endl;
    ss << "u: " << std::endl << u << std::endl << "---" << std::endl;
    ss << "Q: " << std::endl << Q << std::endl << "---" << std::endl;
    ss << "H: " << std::endl << H << std::endl << "---" << std::endl;
    ss << "R: " << std::endl << R << std::endl << "---" << std::endl;
    std::cout << ss.str() << std::endl;

    // TODO: validate
    // matrices and vectors empty
    // appropriate sizes of matrices and vectors

    validated = true;
  }

  // -----------------------------------------
  // IEstimationMethod implementation
  // -----------------------------------------
  Output KalmanFilter::estimate (Input next)
  {
    if (!validated)
      return out;

    // extract the values of next (the measurements) into a vector
    VectorXd z(next.size());
    for (int i = 0; i < next.size(); i++)
      z[i] = next[i].getValue();
    // TODO: save memory and time:
    //VectorXd z = Map<VectorXd>(&next.getValues()[0]);
    
    // Kalman Filtering ------------------------------------------
    // predict (time-update)
    VectorXd x_apriori = A*x + B*u;
    MatrixXd P_apriori = A*P*A.transpose() + Q;
    
    // update (measurement-update)
    K = (P_apriori*H.transpose()) * (H*P_apriori*H.transpose() + R).inverse();  
    x = x_apriori + K*(z - H*x_apriori);
    P = P_apriori - K*H*P_apriori;
    // -----------------------------------------------------------

    std::cout << "x = " << x[0] << std::endl;

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

  void KalmanFilter::init(void)
  {
    // TODO
    /*
    // new state = old state
    x << 0;
    A << 1;

    // no control input
    u << 0;
    B << 0;

    // measurement = state
    H << 1;

    // noise covariance
    Q << 0.2;	// low covariance, means we are sure of our process
		// model
    R << 1;	// variance of the sensor

    // initial error covariance
    P << 1;
    */
  }

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
