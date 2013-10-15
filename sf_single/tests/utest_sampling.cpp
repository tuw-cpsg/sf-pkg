#include <gtest/gtest.h>
#include <stdexcept>
#include <Eigen/Dense>

// testing following API
#include "probability/sampling.h"

using namespace Eigen;
using namespace probability;

namespace SamplingTest 
{
  void sampleAndApproximate(VectorXd mean, MatrixXd covariance)
  {
    int N = 10000;
    
    // sum up for approximating the mean and variance
    VectorXd meanApprox = VectorXd::Zero(mean.size());
    MatrixXd covarianceApprox = MatrixXd::Zero(covariance.rows(), covariance.cols());

    // create N samples
    for (int i = 0; i < N; i++)
    {
      VectorXd sample = sampleNormalDistribution(mean, covariance);

      // accumulate for mean
      meanApprox = meanApprox + sample;

      // accumulate for variance (take true mean for calculation
      // otherwise we would have to save the samples)
      covarianceApprox = covarianceApprox + (sample - mean)*((sample - mean).transpose());
    }

    meanApprox = meanApprox / N;
    covarianceApprox = covarianceApprox / N;

    for (int i = 0; i < mean.size(); i++)
    {
      EXPECT_NEAR(mean[i], meanApprox[i], 0.1);
      for (int j = 0; j < covariance.cols(); j++)
    	EXPECT_NEAR(covariance(i,j), covarianceApprox(i,j), 0.1);
    }
  }

  // -----------------------------------------
  // tests
  // -----------------------------------------
  TEST(SamplingTest, sampleGaussian)
  {
    // random vector: size = 1 (like a random variable) ---------------
    VectorXd mean1(1); 
    mean1 << 0;
    MatrixXd variance1(1,1); 
    variance1 << 1;
    EXPECT_NO_THROW(sampleAndApproximate(mean1, variance1));

    // random vector: size = 3 ----------------------------------------
    VectorXd mean2(3); 
    mean2 << -1, 0, 1;
    MatrixXd covariance2(3,3);
    covariance2 << 
      0.5, 0, 0, 
        0, 1, 0,
        0, 0, 2;
    EXPECT_NO_THROW(sampleAndApproximate(mean2, covariance2));

    // random vector: size = 3, non-diagonal --------------------------
    VectorXd mean3(3); 
    mean3 << -5, 1, 100;
    MatrixXd covariance3(3,3);
    covariance3 << 
        3, 0.1, 0.1, 
      0.1,   1, 0.9,
      0.1, 0.9,   2;
    //EXPECT_NO_THROW(sampleAndApproximate(mean3, covariance3));
    EXPECT_NO_THROW(sampleAndApproximate(mean3, covariance3));

    // random vector: size = 3, non-diagonal, NOT positive definite! --
    VectorXd mean4(2); 
    mean4 << -1000, 0;
    MatrixXd covariance4(2,2);
    covariance4 << 
      1, 2,
      2, 1;
    // throws error (cholesky decomposition failed)
    EXPECT_ANY_THROW(sampleAndApproximate(mean4, covariance4));	
  }
}
