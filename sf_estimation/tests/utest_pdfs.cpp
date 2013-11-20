#include <gtest/gtest.h>
#include <Eigen/Dense>

// testing following API
#include "probability/pdfs.h"

using namespace Eigen;
using namespace probability;

namespace PdfsTest 
{
  // -----------------------------------------
  // tests
  // -----------------------------------------
  TEST(PdfsTest, pdfNormalDistribution)
  {
    // reference results calculated with WolframAlpha
    
    // random vector: size = 1 (like a random variable) ---------------
    VectorXd mean1(1); 
    mean1 << 0;
    MatrixXd variance1(1,1); 
    variance1 << 1;
    VectorXd x1(1); 
    x1 << 0;
    double p1;
    EXPECT_NO_THROW(p1 = pdfNormalDistribution(x1, mean1, variance1));
    EXPECT_NEAR(p1, 0.398942, 0.000001);

    // random vector: size = 2 ----------------------------------------
    VectorXd mean2(2); 
    mean2 << -1, 1;
    MatrixXd covariance2(2,2);
    covariance2 << 
        1, 0.1, 
      0.1,   4;
    VectorXd x2(2); 
    x2 << 0,0;
    double p2;
    EXPECT_NO_THROW(p2 = pdfNormalDistribution(x2, mean2, covariance2));
    EXPECT_NEAR(p2, 0.041527, 0.000001);

    // random vector: size = 3 ----------------------------------------
    VectorXd mean3(3); 
    mean3 << -1, 0, 1;
    MatrixXd covariance3(3,3);
    covariance3 << 
      1, 0, 0, 
      0, 9, 0,
      0, 0, 16;
    VectorXd x3(3); 
    x3 << 0,0,0;
    double p3;
    EXPECT_NO_THROW(p3 = pdfNormalDistribution(x3, mean3, covariance3));
    EXPECT_NEAR(p3, 0.003110, 0.000001);

    // // speed test
    // // calculating the matrix inverse takes time!
    // // inverting every cycle: 52ms
    // // inverting first cycle:  8ms 
    // for (int i = 0; i < 1000; i++)
    //   pdfNormalDistribution(x2, mean2, covariance2);
    // for (int i = 0; i < 500; i++)
    // {
    //   pdfNormalDistribution(x2, mean2, covariance2);
    //   pdfNormalDistribution(x3, mean3, covariance3);
    // }
  }
}
