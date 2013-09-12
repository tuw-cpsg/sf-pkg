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

namespace ExtendedKalmanFilterTest 
{
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

  void f2(VectorXd& x, const VectorXd& u)
  {
    VectorXd x_apriori(x.size());

    x_apriori[0] = x[0] + 0.1*x[1];
    x_apriori[1] = x[1];
    
    x = x_apriori;
  }
  void df2(MatrixXd& A, const VectorXd& x, const VectorXd& u)
  {
    A(0,0) = 1;
    A(0,1) = 0.1;
    A(1,0) = 0;
    A(1,1) = 1;
  }
  void h2(VectorXd& z, const VectorXd& x)
  {
    z[0] = x[1];
  }
  void dh2(MatrixXd& H, const VectorXd& x)
  {
    H(0,0) = 0;
    H(0,1) = 1;
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

    // check validation itself (e.g. invalid sizes of vectors/matrices,
    // empty callbacks, etc.)
    ekf.setStateTransitionModel(0);
    EXPECT_THROW(ekf.validate(), runtime_error);		// "STM missing"
    ekf.setStateTransitionModel(f);
    ekf.setJacobianOfStateTransitionModel(0);	
    EXPECT_THROW(ekf.validate(), runtime_error);		// "STMJ missing"
    ekf.setJacobianOfStateTransitionModel(df);
    ekf.setObservationModel(0);
    EXPECT_THROW(ekf.validate(), runtime_error);		// "OM missing"
    ekf.setObservationModel(h);
    ekf.setJacobianOfObservationModel(0);
    EXPECT_THROW(ekf.validate(), runtime_error);		// "OMJ missing"
    ekf.setJacobianOfObservationModel(dh);

    EXPECT_THROW(ekf.validate(), runtime_error);		// "PNC missing"
    vector< vector<double> > Q = { {0.1} };
    ekf.setProcessNoiseCovariance(Q);
    EXPECT_THROW(ekf.validate(), runtime_error);		// "MNC missing"
    vector< vector<double> > R = { {10} };
    ekf.setMeasurementNoiseCovariance(R);

    EXPECT_THROW(ekf.validate(), runtime_error);		// "state-size missing"
    ekf.setSizeOfState(2);

    vector< vector<double> > Q_f = { {0.1,1} };
    ekf.setProcessNoiseCovariance(Q_f);
    EXPECT_THROW(ekf.validate(), runtime_error);		// "Q: invalid size"
    ekf.setProcessNoiseCovariance(Q);
    vector< vector<double> > R_f = { {10} , {-1} };
    ekf.setMeasurementNoiseCovariance(R_f);
    EXPECT_THROW(ekf.validate(), runtime_error);		// "R: invalid size"
    ekf.setMeasurementNoiseCovariance(R);

    // optional params
    vector<double> u_f = {0.1,1};
    ekf.setControlInput(u_f);
    EXPECT_THROW(ekf.validate(), runtime_error);		// "u: invalid size"
    vector<double> u = {0};
    ekf.setControlInput(u);
    vector< vector<double> > P0_f = { {0.1,1} };
    ekf.setInitialErrorCovariance(P0_f);
    EXPECT_THROW(ekf.validate(), runtime_error);		// "P0: invalid size"
    vector< vector<double> > P0 = { {1} };
    ekf.setInitialErrorCovariance(P0);
    EXPECT_THROW(ekf.validate(), runtime_error);		// "P0: invalid size"

    ekf.setSizeOfState(1);
    EXPECT_NO_THROW(ekf.validate());
  }

  TEST(ExtendedKalmanFilterTest, validationEffect)
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
  
    // EKF should now be released and return an estimate (should not be
    // the default)
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
  
    EXPECT_EQ(out.size(), 1);
    EXPECT_NEAR(out[0].getValue(), 0.0099, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.0990, 0.0001);

    in[0].setValue(5);
    out = ekf.estimate(in);
  
    EXPECT_NEAR(out[0].getValue(), 0.1073, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.1951, 0.0001);

    ExtendedKalmanFilter ekf2;
    ekf2.setStateTransitionModel(f2);
    ekf2.setJacobianOfStateTransitionModel(df2);
    ekf2.setObservationModel(h2);
    ekf2.setJacobianOfObservationModel(dh2);
    ekf2.setProcessNoiseCovariance(Q);
    ekf2.setMeasurementNoiseCovariance(R);
    ekf2.setSizeOfState(2);

    ekf2.validate();

    in[0].setValue(1);
    out = ekf2.estimate(in);
    EXPECT_NEAR(out[0].getValue(), 0.1073, 0.0001);
    EXPECT_NEAR(out[1].getValue(), 0.1073, 0.0001);
    EXPECT_EQ(out.size(), 2);
  }

}
