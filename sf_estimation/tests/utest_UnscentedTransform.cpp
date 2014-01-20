#include <gtest/gtest.h>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include <cmath>

// testing following API
#include "estimation/UnscentedTransform.h"
#include "estimation/ITransformer.h"

using namespace std;
using namespace estimation;

namespace UnscentedTransformTest 
{
  class TestTransformer : public ITransformer
  {
  private:
    int choice;

    VectorXd f1(const VectorXd& x) { return x; }
    VectorXd f2(const VectorXd& x) { return 2*x; }
    VectorXd f3(const VectorXd& x) { 
      VectorXd y = VectorXd::Zero(2); 
      y[0] = x[0];
      y[1] = x[1];
      return y;
    }
    VectorXd f4(const VectorXd& x) {	// nonlinear example
      VectorXd y(x.size());
      for (int i = 0; i < x.size(); i++)
	y[i] = sin(x[i]);
      return y; 
    }

  public:
    TestTransformer(int choice) { this->choice = choice; }
    ~TestTransformer() { }

    VectorXd transform(const VectorXd& x)
    {
      switch(choice) {
      case 1: return f1(x);
      case 2: return f2(x);
      case 3: return f3(x);
      case 4: return f4(x);
      default: return f1(x);
      }
    }
  };

  // -----------------------------------------
  // tests
  // -----------------------------------------
  TEST(UnscentedTransformTest, initialization)
  {
    VectorXd x(3);
    x << 1,2,3;
    MatrixXd Px(3,3);
    Px << 1,0,0 , 0,1,0 , 0,0,1;
    TestTransformer tt1(1);

    EXPECT_NO_THROW(UnscentedTransform ut_t1(x, Px, &tt1));
    
    // missing transformer
    EXPECT_THROW(UnscentedTransform ut_t1(x, Px, 0), runtime_error);

    // invalid sizes
    VectorXd x_f(4);
    x_f << 1,2,3,4;
    MatrixXd Px_f(2,3);
    Px_f << 1,0,0 , 0,1,0;

    EXPECT_THROW(UnscentedTransform ut1(x_f, Px, &tt1), length_error);
    EXPECT_THROW(UnscentedTransform ut2(x, Px_f, &tt1), length_error);
    EXPECT_THROW(UnscentedTransform ut3(x_f, Px_f, &tt1), length_error);

    // check sizes of output
    UnscentedTransform ut_t1(x, Px, &tt1);
    EXPECT_EQ(ut_t1.mean().size(), 3);
    EXPECT_EQ(ut_t1.covariance().rows(), 3);
    EXPECT_EQ(ut_t1.covariance().cols(), 3);
    EXPECT_EQ(ut_t1.crossCovarianceXY().rows(), 3);
    EXPECT_EQ(ut_t1.crossCovarianceXY().cols(), 3);

    TestTransformer tt3(3);
    UnscentedTransform ut_t3(x, Px, &tt3);
    EXPECT_EQ(ut_t3.mean().size(), 2);
    EXPECT_EQ(ut_t3.covariance().rows(), 2);
    EXPECT_EQ(ut_t3.covariance().cols(), 2);
    EXPECT_EQ(ut_t3.crossCovarianceXY().rows(), 3);
    EXPECT_EQ(ut_t3.crossCovarianceXY().cols(), 2);	
  }

  TEST(UnscentedTransformTest, functionality)
  {
    VectorXd x(3);
    x << 1,-2,3;
    MatrixXd Px(3,3);
    Px << 1,0,0 , 0,1,0 , 0,0,1;
    TestTransformer tt1(1);
    TestTransformer tt2(2);
    TestTransformer tt3(3);

    // &tt1: y = x, so mean and covariance should remain the same

    UnscentedTransform ut(x, Px, &tt1);
    VectorXd y;
    MatrixXd Py, Pxy;
    
    EXPECT_NO_THROW(ut.compute());
    EXPECT_NO_THROW(y = ut.mean());
    EXPECT_NO_THROW(Py = ut.covariance());
    EXPECT_NO_THROW(Pxy = ut.crossCovarianceXY());

    ASSERT_EQ(x.size(), y.size());

    for (int i = 0; i < y.size(); i++)
      EXPECT_NEAR(x[i], y[i], 0.000001);

    ASSERT_EQ(x.rows(), y.rows());
    ASSERT_EQ(x.cols(), y.cols());

    for (int r = 0; r < Py.rows(); r++)
      for (int c = 0; c < Py.cols(); c++) {
	EXPECT_NEAR(Px(r,c), Py(r,c), 0.000001);
	EXPECT_NEAR(Px(r,c), Pxy(r,c), 0.000001);
      }

    // &tt2: y = 2x, so mean doubled, covariance*4, cross-covariance*2

    UnscentedTransform ut2(x, Px, &tt2);
    VectorXd y2;
    MatrixXd Py2, Pxy2;
    EXPECT_NO_THROW(ut2.compute());
    EXPECT_NO_THROW(y2 = ut2.mean());
    EXPECT_NO_THROW(Py2 = ut2.covariance());
    EXPECT_NO_THROW(Pxy2 = ut2.crossCovarianceXY());

    ASSERT_EQ(x.size(), y.size());

    for (int i = 0; i < y2.size(); i++)
      EXPECT_NEAR(2*x[i], y2[i], 0.000001);

    ASSERT_EQ(x.rows(), y.rows());
    ASSERT_EQ(x.cols(), y.cols());

    for (int r = 0; r < Py2.rows(); r++)
      for (int c = 0; c < Py2.cols(); c++) {
	EXPECT_NEAR(4*Px(r,c), Py2(r,c), 0.000001);
	EXPECT_NEAR(2*Px(r,c), Pxy2(r,c), 0.000001);
      }

    // &tt3: y = x[0,1], so mean/cov/.. remains the same but cut off

    UnscentedTransform ut3(x, Px, &tt3);
    VectorXd y3;
    MatrixXd Py3, Pxy3;
    EXPECT_NO_THROW(ut3.compute());
    EXPECT_NO_THROW(y3 = ut3.mean());
    EXPECT_NO_THROW(Py3 = ut3.covariance());
    EXPECT_NO_THROW(Pxy3 = ut3.crossCovarianceXY());

    EXPECT_EQ(y3.size(),2);
    for (int i = 0; i < y3.size(); i++)
      EXPECT_NEAR(x[i], y3[i], 0.000001);

    for (int r = 0; r < Py3.rows(); r++)
      for (int c = 0; c < Py3.cols(); c++)
	EXPECT_NEAR(Px(r,c), Py3(r,c), 0.000001);

    EXPECT_EQ(Pxy3.rows(), 3);
    EXPECT_EQ(Pxy3.cols(), 2);
    for (int r = 0; r < Pxy3.rows(); r++)
      for (int c = 0; c < Pxy3.cols(); c++)
	EXPECT_NEAR(Px(r,c), Pxy3(r,c), 0.000001);
  }

  TEST(UnscentedTransformTest, functionalityNonlinear)
  {
    VectorXd x(4);
    x << 10*M_PI/180, 30*M_PI/180, 60*M_PI/180, 90*M_PI/180;	// 30°, 0°, 90°
    MatrixXd Px(4,4);
    Px << 1,0,0,0 , 0,0.1,0,0 , 0,0,0.01,0 , 0,0,0,0.001;

    TestTransformer tt4(4);

    // tt4: y[i] = sin(x[i])

    UnscentedTransform ut(x, Px, &tt4);
    VectorXd y;
    MatrixXd Py, Pxy;
    
    EXPECT_NO_THROW(ut.compute());
    EXPECT_NO_THROW(y = ut.mean());
    EXPECT_NO_THROW(Py = ut.covariance());
    EXPECT_NO_THROW(Pxy = ut.crossCovarianceXY());

    ASSERT_EQ(x.size(), y.size());

    for (int i = 0; i < y.size(); i++) {
      EXPECT_NEAR(sin(x[i]), y[i], Px(i,i));
    }

/*
    // TODO

    ASSERT_EQ(x.rows(), y.rows());
    ASSERT_EQ(x.cols(), y.cols());

    for (int r = 0; r < Py.rows(); r++)
      for (int c = 0; c < Py.cols(); c++) {
	EXPECT_NEAR(Px(r,c), Py(r,c), 0.000001);
	EXPECT_NEAR(Px(r,c), Pxy(r,c), 0.000001);
      }
*/
  }

  TEST(UnscentedTransformTest, examples)
  {
    int n;
    VectorXd x, y;
    MatrixXd Px, Py;

    // example 1 -----------------------
    n = 1;
    x = VectorXd::Zero(n);
    Px = MatrixXd::Identity(n,n);

    TestTransformer tt1(1);
    UnscentedTransform ut_ex1_s(x, Px, &tt1);
    ut_ex1_s.compute();
    y = ut_ex1_s.mean();
    Py = ut_ex1_s.covariance();
    /*
    cout << "example 1 (predict)" << endl;
    cout << "y: " << endl << y << endl;
    cout << "Py: " << endl << Py << endl; 
    */

    MatrixXd Q = 0.1 * MatrixXd::Identity(n,n);
    UnscentedTransform ut_ex1_m(x, Px+Q, &tt1);
    ut_ex1_m.compute();
    /*
    cout << "example 1 (correct)" << endl;
    cout << "y: " << endl << ut_ex1_m.mean() << endl;
    cout << "Py: " << endl << ut_ex1_m.covariance() << endl; 
    cout << "Pxy: " << endl << ut_ex1_m.crossCovarianceXY() << endl;
    */
  }
}
