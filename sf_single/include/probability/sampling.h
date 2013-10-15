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
   * @note When sampling with non-diagonal covariance this function
   * might be inefficient (Cholesky decomposition is applied everytime
   * the covariance changes).
   * @note The covariance must be symmetric and positive definite.
   */
  VectorXd sampleNormalDistribution(VectorXd mean, MatrixXd covariance);
}

#endif
