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
  std::vector<double> AbstractKalmanFilter::getState () const
  {
    std::vector<double> vx;
    
    // copy current state to std::vector
    for (int i = 0; i < x.size(); i++)
      vx.push_back(x[i]);

    return vx;
  }
  
  void AbstractKalmanFilter::setInitialState (std::vector<double>& x0)
  {
    copy(x0, this->x);
    validated = false;
  }

  void AbstractKalmanFilter::setInitialErrorCovariance(std::vector< std::vector<double> >& P0)
  {
    copy(P0, this->P);
    validated = false;
  }

  void AbstractKalmanFilter::setControlInput (std::vector<double>& u) 
  {
    copy(u, this->u);
    validated = false;
  }

  void AbstractKalmanFilter::setProcessNoiseCovariance (std::vector< std::vector<double> >& Q)
  {
    copy(Q, this->Q);
    validated = false;
  }

  void AbstractKalmanFilter::setMeasurementNoiseCovariance (std::vector< std::vector<double> >& R)
  {
    copy(R, this->R);
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
  void AbstractKalmanFilter::copy(std::vector<double>& src, VectorXd& dest)
  {
    int rows = src.size();
    
    // specify size of destination matrix according src
    dest.resize(rows);		// needed, if dest-vector is empty

    // copy src to dest
    for (int row = 0; row < rows; row++)
      dest(row) = src[row];
  }

  void AbstractKalmanFilter::copy(std::vector< std::vector<double> >& src, MatrixXd& dest)
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
