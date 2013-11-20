/**
 * @file 
 * @author Denise Ratasich
 * @date 14.10.2013
 *
 * @brief Implement sampling from specific distributions.
 *
 * Currently only Gaussian distributions can be choosen for process
 * and covariance noise. Hence only sampling from a Gaussian
 * distribution is implemented.
 */

#include "probability/pdfs.h"

#include <stdexcept>
#define _USE_MATH_DEFINES
#include <cmath>

#ifdef M_PI
  #define PI	M_PI
#else
  #define PI	(3.14159265358979)
#endif

namespace probability
{
  double pdfNormalDistribution(VectorXd x, VectorXd mean, MatrixXd covariance)
  {
    static MatrixXd cov;
    static MatrixXd covInverse;
    static double norm;
    double likelihood;

    // speed test results of calling this function 1000 times with
    // x=(0,0,0), mean=(-1,0,1), cov=((1,0,0),(0,9,0),(0,0,16)):
    // calculating the matrix inverse takes time!  
    // inverting every cycle:      52ms 
    // inverting only first cycle:  8ms
    if (cov.rows() != covariance.rows()  ||
	cov.cols() != covariance.cols()  ||
	cov != covariance)
    {
      // calculate normalizing factor
      norm = pow((2*PI),covariance.rows()) * covariance.determinant();
      norm = 1 / sqrt(norm);

      covInverse = covariance.inverse();

      cov = covariance;		// save
    }

    // calculate the scalar in the exponent
    x = x - mean;
    likelihood = x.transpose() * covInverse * x;
    likelihood = exp(-0.5 * likelihood);

    // normalize
    likelihood *= norm;

    return likelihood;
  }
}

#undef PI
#undef _USE_MATH_DEFINES
