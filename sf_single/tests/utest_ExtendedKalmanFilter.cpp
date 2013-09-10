#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>

// testing following API
#include "estimation/ExtendedKalmanFilter.h"
#include "estimation/Input.h"
#include "estimation/InputValue.h"
#include "estimation/OutputValue.h"

using namespace std;
using namespace estimation;

// test ekf
void f(VectorXd& x, const VectorXd& u)
{
  x[0] = x[0];
}
void df(MatrixXd& A, const VectorXd& x, const VectorXd& u)
{
  A(0,0) = 1;
}
void h(VectorXd& z, const VectorXd& x)
{
  z[0] = x[0];
}
void dh(MatrixXd& H, const VectorXd& x)
{
  H(0,0) = 1;
}

// -----------------------------------------
// tests
// -----------------------------------------
TEST(ExtendedKalmanFilterTest, initialization)
{
  ExtendedKalmanFilter ekf;

  // check setting required params
  EXPECT_THROW(ekf.validate(), runtime_error);	// "STM missing"
  ekf.setStateTransitionModel(f);
  EXPECT_THROW(ekf.validate(), runtime_error);	// "Jacobian of STM missing"
  ekf.setJacobianOfStateTransitionModel(df);
  EXPECT_THROW(ekf.validate(), runtime_error);	// "OM missing"
  ekf.setObservationModel(h);
  EXPECT_THROW(ekf.validate(), runtime_error);	// "Jacobian of OM missing"
  ekf.setJacobianOfObservationModel(dh);
  EXPECT_THROW(ekf.validate(), runtime_error);	// "PNC missing"
  vector< vector<double> > Q = { {0.1} };
  ekf.setProcessNoiseCovariance(Q);
  EXPECT_THROW(ekf.validate(), runtime_error);	// "MNC missing"
  vector< vector<double> > R = { {10} };
  ekf.setMeasurementNoiseCovariance(R);
  EXPECT_THROW(ekf.validate(), runtime_error);	// "Size of state cannot be evaluated"
  ekf.setSizeOfState(1);
  EXPECT_NO_THROW(ekf.validate());		// all required params given

  // check setting optional params
  vector<double> u_f = { 0,1 };
  ekf.setControlInput(u_f);
  EXPECT_THROW(ekf.validate(), runtime_error);	// "control input has invalid size"
  vector<double> u = { 0 };
  ekf.setControlInput(u);
  EXPECT_NO_THROW(ekf.validate());
  
  vector< vector<double> > P0_f = { {1,0} , {0,1} };
  ekf.setInitialErrorCovariance(P0_f);
  EXPECT_THROW(ekf.validate(), runtime_error);	// "EC has invalid size"
  vector< vector<double> > P0 = { {0} };
  ekf.setInitialErrorCovariance(P0);

  // final successful validation
  EXPECT_NO_THROW(ekf.validate());

  // check initialization of output
  Output out = ekf.getLastEstimate();
  OutputValue defaultOutVal;
  EXPECT_GT(out.size(), 0);
  EXPECT_DOUBLE_EQ(out.getValue(), defaultOutVal.getValue());
}

TEST(ExtendedKalmanFilterTest, validation)
{
  ExtendedKalmanFilter ekf;

  ekf.setStateTransitionModel(f);
  ekf.setJacobianOfStateTransitionModel(df);
  ekf.setObservationModel(h);
  ekf.setJacobianOfObservationModel(dh);
  vector< vector<double> > Q = { {0.1} };
  ekf.setProcessNoiseCovariance(Q);
  vector< vector<double> > R = { {10} };
  ekf.setMeasurementNoiseCovariance(R);
  ekf.setSizeOfState(1);
  
  // validate has an effect?
  InputValue measurement(1);
  Input in(measurement);
  Output out;
  
  EXPECT_THROW(out = ekf.estimate(in), runtime_error);	// "not yet validated"
  
  EXPECT_NO_THROW(ekf.validate());
  
  // EKF should now be released and return an estimate (should not be the default)
  EXPECT_NO_THROW(out = ekf.estimate(in));
  EXPECT_GT(out.size(), 0);
  OutputValue defaultOutVal;
  EXPECT_NE(out.getValue(), defaultOutVal.getValue());
}

TEST(ExtendedKalmanFilterTest, functionality)
{
  ExtendedKalmanFilter ekf;

  ekf.setStateTransitionModel(f);
  ekf.setJacobianOfStateTransitionModel(df);
  ekf.setObservationModel(h);
  ekf.setJacobianOfObservationModel(dh);
  vector< vector<double> > Q = { {0.1} };
  ekf.setProcessNoiseCovariance(Q);
  vector< vector<double> > R = { {10} };
  ekf.setMeasurementNoiseCovariance(R);
  ekf.setSizeOfState(1);

  ekf.validate();
  
  // check if the calculation of an estimate is correct
  InputValue measurement(1);
  Input in(measurement);
  Output out = ekf.estimate(in);
  
  EXPECT_NEAR(out[0].getValue(), 0.0099, 0.0001);
  EXPECT_NEAR(out[0].getVariance(), 0.0990, 0.0001);

  in[0].setValue(5);
  out = ekf.estimate(in);
  
  EXPECT_NEAR(out[0].getValue(), 0.1073, 0.0001);
  EXPECT_NEAR(out[0].getVariance(), 0.1951, 0.0001);
}
