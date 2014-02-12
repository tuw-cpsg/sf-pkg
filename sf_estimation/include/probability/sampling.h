/**
 * @file 
 * @author Denise Ratasich
 * @date 14.10.2013
 *
 * @brief Header file of sampling methods.
 */

#ifndef __ESTIMATION_SAMPLING_H__
#define __ESTIMATION_SAMPLING_H__

#include <Eigen/Dense>
using namespace Eigen;

namespace probability
{
  /**
   * @brief Samples a random vector with normal distribution.
   *
   * @param mean The mean of the normal distribution.
   * @param covariance The covariance of the normal distribution.
   * @return The sample x ~ N(mean,covariance).
   *
   * @note When sampling with non-diagonal covariance this function
   * might be inefficient (Cholesky decomposition is applied everytime
   * the covariance changes).
   * @note The covariance must be symmetric and positive definite.
   */
  VectorXd sampleNormalDistribution(VectorXd mean, MatrixXd covariance);

  /**
   * @brief Samples a random vector with uniform distribution.
   *
   * The probability of a continious uniform distribution is
   * \f$1/|R|\f$ for each sample, where \f$R \subseteq R^N\f$ is the
   * region under the pdf.
   *
   * @param a The lower bound of the uniform distribution.
   * @param b The upper bound of the uniform distribution.
   * @return The sample x ~ U(a,b).
   *
   * @note The random variables are assumed to be independent,
   * i.e. each sample is constructed of random numbers in between the
   * given bounds a and b. In the two-dimensional case this means the
   * area where x can occur is a rectangle, and can't be e.g. a
   * circle.
   */
  VectorXd sampleUniformDistribution(VectorXd a, VectorXd b);
}

#endif
