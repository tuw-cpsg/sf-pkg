/**
 * @file 
 * @author Denise Ratasich
 * @date 17.09.2013
 *
 * @brief Implementation of an Unscented Transform.
 */

#include "estimation/UnscentedTransform.h"

#include <stdexcept>
#include <vector>
#include <cmath>
#include <Eigen/Cholesky>

namespace estimation 
{
  UnscentedTransform::UnscentedTransform (VectorXd x, MatrixXd Px, ITransformer* transformer)
  {    
    // validation of params
    if (x.size() != Px.rows()  ||  x.size() != Px.cols())
      throw std::length_error("Initialization of Unscented Transform failed. "
			      "Invalid size of covariance, doesn't match that of vector x.");

    if (transformer == 0)
      throw std::runtime_error("Initialization of Unscented Transform failed. "
			       "No transformer passed.");

    // initializes parameters
    this->x = x;
    this->Px = Px;
    this->transformer = transformer;

    // set default scaling factors
    alpha = 1;			// spread
    beta = 2;			// distribution = Gaussian
    kappa = 3 - x.size();	// TODO whatever?!

    // initialize output; to get the size of y, we must propagate a
    // vector through the transformer
    VectorXd testY = transformer->transform(VectorXd::Zero(x.size()));
    y = VectorXd::Zero(testY.size());
    Py = MatrixXd::Zero(testY.size(),testY.size());
    Pxy = MatrixXd::Zero(x.size(),testY.size());
  }

  UnscentedTransform::~UnscentedTransform () 
  {
    // no space to free
  }

  void UnscentedTransform::compute (void)
  {
    std::vector<double> weights;
    std::vector<VectorXd> sigmaPoints_prior;
    std::vector<VectorXd> sigmaPoints_post;

    try
    {
      // fill array sigmaPoints_prior with sigma points
      generateSigmaPoints(sigmaPoints_prior, weights);

      // pop last element because its not for calculating the mean;
      // popped value will replace first element when calculating the
      // covariance
      double w0_cov = weights.back();
      weights.pop_back();

      // calculate mean y -----------------------------------
      for (int i = 0; i < sigmaPoints_prior.size(); i++)
      {
	// propagate the sigma points through the function and save
	// (needed for covariance) and covariance cannot be accumulated
	// here too because the mean is needed :(
	sigmaPoints_post.push_back(transformer->transform(sigmaPoints_prior[i]));
    
	// accumulate to the approximated mean
	y += weights[i] * sigmaPoints_post[i];
      }
      // calculate covariances Py, Pxy ----------------------
      weights[0] = w0_cov;			// prepare weights
      VectorXd t(sigmaPoints_post[0].size());	// help vector (for transform)

      for (int i = 0; i < sigmaPoints_post.size(); i++)
      {
	t = sigmaPoints_post[i] - y;
	Py += weights[i] * (sigmaPoints_post[i] - y) * t.transpose();
	Pxy += weights[i] * (sigmaPoints_prior[i] - x) * t.transpose();
      }
    } 
    catch (std::exception& e) 
    {
      std::string additionalInfo = "Computation of Unscented Transform failed. ";
      throw std::runtime_error(additionalInfo + e.what());
    }
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  
  VectorXd UnscentedTransform::mean (void) const
  {
    return y;
  }
  
  MatrixXd UnscentedTransform::covariance (void) const
  {
    return Py;
  }
  
  MatrixXd UnscentedTransform::crossCovarianceXY (void) const
  {
    return Pxy;
  }

  // -----------------------------------------
  // private functions
  // -----------------------------------------
  void UnscentedTransform::generateSigmaPoints (std::vector<VectorXd>& sigmaPoints, std::vector<double>& weights)
  {
    int L = x.size();

    // calculate values which are needed more often
    double alphaPow2 = pow(alpha,2);
    double lambda = alphaPow2 * (L + kappa) - L;
    double lPlusLambda = L + lambda;

    // calculate matrix square root ----------------
    
    // matrix which should form the sigma points (matrix to take the
    // root of)
    MatrixXd msr(L,L);
    msr = lPlusLambda * Px;

    // calculate the root of the matrix; this is done here by standard
    // Cholesky decomposition (this works for symmetric,
    // positive-definite matrices, which Px is like)
    msr = msr.llt().matrixL();

    // generate sigma points -----------------------
    VectorXd col(x.size());

    // chi_0
    sigmaPoints.push_back(x);

    // chi_i = x + msr_i-1, i=1,..,L
    for (int i = 1; i <= L; i++) 
      sigmaPoints.push_back(x + msr.col(i-1));

    // chi_i = x - msr_i-L-1, i=L+1,..,2L
    for (int i = 1; i <= L; i++) 
      sigmaPoints.push_back(x - msr.col(i-1));

    // set weights --------------------------------

    // W_0 for mean
    weights.push_back(lambda / lPlusLambda);

    // W_1, W_2, ..., W_2L for mean AND covariance
    for (int i = 1; i <= 2*L; i++)
      weights.push_back(1 / (2 * lPlusLambda));
    
    // W_0 for covariance
    weights.push_back(lambda/lPlusLambda + 1 - alphaPow2 + beta);
  }
}
