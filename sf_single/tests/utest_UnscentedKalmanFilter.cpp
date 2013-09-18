#include <gtest/gtest.h>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>

// testing following API
#include "estimation/UnscentedKalmanFilter.h"

using namespace std;
using namespace estimation;

namespace UnscentedKalmanFilterTest 
{
  void f(VectorXd& x, const VectorXd& u)
  {
    // x = x
  }
  void h(VectorXd& z, const VectorXd& x)
  {
    z = x;
  }

  void f2(VectorXd& x, const VectorXd& u)
  {
    VectorXd x_apriori(x.size());

    x_apriori[0] = x[0] + 0.1*x[1];
    x_apriori[1] = x[1];
    
    x = x_apriori;
  }
  void h2(VectorXd& z, const VectorXd& x)
  {
    z[0] = x[1];
  }

  // -----------------------------------------
  // tests
  // -----------------------------------------
  TEST(UnscentedKalmanFilterTest, initialization)
  {
    UnscentedKalmanFilter ukf;

    // check setting required params
    EXPECT_THROW(ukf.validate(), IEstimator::estimator_error);	// "STM missing"
    ukf.setStateTransitionModel(f);
    EXPECT_THROW(ukf.validate(), IEstimator::estimator_error);	// "OM missing"
    ukf.setObservationModel(h);
    EXPECT_THROW(ukf.validate(), IEstimator::estimator_error);	// "PNC missing"
    vector< vector<double> > Q = { {0.1} };
    ukf.setProcessNoiseCovariance(Q);
    EXPECT_THROW(ukf.validate(), IEstimator::estimator_error);	// "MNC missing"
    vector< vector<double> > R = { {10} };
    ukf.setMeasurementNoiseCovariance(R);
    EXPECT_NO_THROW(ukf.validate());			// all required params given

    // check setting optional params
    vector<double> u = { 0 };
    ukf.setControlInput(u);
    EXPECT_NO_THROW(ukf.validate());

    // TODO more validation if not changed...

    // check initialization of output
    Output out = ukf.getLastEstimate();
    OutputValue defaultOutVal;
    EXPECT_GT(out.size(), 0);
    EXPECT_DOUBLE_EQ(out.getValue(), defaultOutVal.getValue());
  }

/*
  TEST(UnscentedKalmanFilterTest, validation)
  {
    // TODO more validation if not changed...
  }

  TEST(UnscentedKalmanFilterTest, validationEffect)
  {
    // TODO more validation if not changed...
  }
*/

  TEST(UnscentedKalmanFilterTest, functionality)
  {
    UnscentedKalmanFilter ukf;
    
    ukf.setStateTransitionModel(f);
    ukf.setObservationModel(h);
    vector< vector<double> > Q = { {0.1} };
    ukf.setProcessNoiseCovariance(Q);
    vector< vector<double> > R = { {10} };
    ukf.setMeasurementNoiseCovariance(R);

    ukf.validate();

    // check if the calculation of an estimate is correct
    InputValue measurement(1);
    Input in(measurement);
    Output out = ukf.estimate(in);

    EXPECT_EQ(out.size(), 1);
    EXPECT_NEAR(out[0].getValue(), 0.0991, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 1.0892, 0.0001);
  }

}
