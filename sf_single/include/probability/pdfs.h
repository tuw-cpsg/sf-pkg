/**
 * @file 
 * @author Denise Ratasich
 * @date 14.10.2013
 *
 * @brief Header file of probability density functions.
 */

#ifndef __ESTIMATION_PDFS_H__
#define __ESTIMATION_PDFS_H__

#include <Eigen/Dense>
using namespace Eigen;

/**
 * @brief Holds functions for working with random vectors.
 */
namespace probability
{
  /**
   * @brief Returns the likelihood for a random variable X ~
   * N(mean,covariance) to take on the concrete value x.
   *
   * pdf of normal distribution: 
   * \f[ f_x(x) = \frac{1}{\sqrt((2 \pi)^N det(C_x))} exp \left( -\frac12 (x-\mu_x)^T C_x^{-1} (x-\mu_x) \right) \f]
   *
   * @param x The concrete value.
   * @param mean The mean of the random variable X.
   * @param covariance The covariance of the random variable X.
   * @return The likelihood of the given concrete value x.
   */
  double pdfNormalDistribution(VectorXd x, VectorXd mean, MatrixXd covariance);
}

#endif
