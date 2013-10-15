#include <gtest/gtest.h>
#include <stdexcept>
#include <cmath>
#include <Eigen/Dense>

// testing following API
#include "probability/sampling.h"

using namespace Eigen;
using namespace probability;

namespace SamplingTest 
{
  void sampleNormalAndApproximate(VectorXd mean, MatrixXd covariance)
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
    	EXPECT_NEAR(covariance(i,j), covarianceApprox(i,j), 0.2);
    }
  }

  VectorXd meanOfUniform(VectorXd a, VectorXd b)
  {
    int N = 1000;
    
    // sum up for approximating the mean and variance
    VectorXd meanApprox = VectorXd::Zero(a.size());

    // create N samples
    for (int i = 0; i < N; i++)
    {
      VectorXd sample = sampleUniformDistribution(a, b);
      meanApprox = meanApprox + sample;	// accumulate for mean
    }

    meanApprox = meanApprox / N;

    return meanApprox;
  }

  // -----------------------------------------
  // tests
  // -----------------------------------------
  TEST(SamplingTest, sampleNormalDistribution)
  {
    // random vector: size = 1 (like a random variable) ---------------
    VectorXd mean1(1); 
    mean1 << 0;
    MatrixXd variance1(1,1); 
    variance1 << 1;
    EXPECT_NO_THROW(sampleNormalAndApproximate(mean1, variance1));

    // random vector: size = 3 ----------------------------------------
    VectorXd mean2(3); 
    mean2 << -1, 0, 1;
    MatrixXd covariance2(3,3);
    covariance2 << 
      0.5, 0, 0, 
        0, 1, 0,
        0, 0, 2;
    EXPECT_NO_THROW(sampleNormalAndApproximate(mean2, covariance2));

    // random vector: size = 3, non-diagonal --------------------------
    VectorXd mean3(3); 
    mean3 << -5, 1, 100;
    MatrixXd covariance3(3,3);
    covariance3 << 
        3, 0.1, 0.1, 
      0.1,   1, 0.9,
      0.1, 0.9,   2;
    EXPECT_NO_THROW(sampleNormalAndApproximate(mean3, covariance3));

    // random vector: size = 3, non-diagonal, NOT positive definite! --
    VectorXd mean4(2); 
    mean4 << -1000, 0;
    MatrixXd covariance4(2,2);
    covariance4 << 
      1, 2,
      2, 1;
    // throws error (cholesky decomposition failed)
    EXPECT_ANY_THROW(sampleNormalAndApproximate(mean4, covariance4));	
  }

  TEST(SamplingTest, sampleUniformDistribution)
  {
    // random vector: size = 1 (like a random variable) ---------------
    VectorXd a1(1); 
    a1 << 0;
    VectorXd b1(1); 
    b1 << 1;
    VectorXd meanApprox;
    EXPECT_NO_THROW(meanApprox = meanOfUniform(a1,b1));
    for (int i = 0; i < meanApprox.size(); i++)
      EXPECT_NEAR((a1[i]+b1[i])/2, meanApprox[i], 0.2);

    // random vector: size = 2 ----------------------------------------
    VectorXd a2(2); 
    a2 << -3, 0;
    VectorXd b2(2);
    b2 << 3, 2;
    EXPECT_NO_THROW(meanApprox = meanOfUniform(a2, b2));
    for (int i = 0; i < meanApprox.size(); i++)
      EXPECT_NEAR((a2[i]+b2[i])/2, meanApprox[i], 0.2);

    // check if really random!
    VectorXd meanApprox1 = meanOfUniform(a2, b2);
    VectorXd meanApprox2 = meanOfUniform(a2, b2);
    EXPECT_GT(fabs(meanApprox1[0] - meanApprox2[0]), 0.000001);
  }
}
