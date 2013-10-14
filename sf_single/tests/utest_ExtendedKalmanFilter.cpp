#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>

// testing following API
#include "estimation/IEstimator.h"
#include "estimation/ExtendedKalmanFilter.h"
#include "estimation/Input.h"
#include "estimation/InputValue.h"
#include "estimation/Output.h"
#include "estimation/OutputValue.h"

using namespace std;
using namespace estimation;

namespace ExtendedKalmanFilterTest 
{
  void f(VectorXd& x, const VectorXd& u)
  {
    x[0] = x[0] + u[0];
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
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "STM missing"
    ekf.setStateTransitionModel(f);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "Jacobian of STM missing"
    ekf.setJacobianOfStateTransitionModel(df);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "OM missing"
    ekf.setObservationModel(h);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "Jacobian of OM missing"
    ekf.setJacobianOfObservationModel(dh);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "PNC missing"
    MatrixXd Q(1,1); Q << 0.1;
    ekf.setProcessNoiseCovariance(Q);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "MNC missing"
    MatrixXd R(1,1); R << 10;
    ekf.setMeasurementNoiseCovariance(R);
    EXPECT_NO_THROW(ekf.validate());			// all required params given

    // check setting optional params  
    // optional params doesn't set new sizes
    MatrixXd P0_f(2,2); P0_f << 1, 0, 0, 1;
    ekf.setInitialErrorCovariance(P0_f);
    EXPECT_NO_THROW(ekf.validate());
    EXPECT_EQ(ekf.getState().size(), 1);

    VectorXd x_f(2); x_f << 0,1;
    ekf.setInitialState(x_f);
    EXPECT_NO_THROW(ekf.validate());
    EXPECT_NE(ekf.getState().size(), 2);

    // required param Q sets new size of x and out
    MatrixXd Q2(2,2); Q2 << 0.1, 0, 0, 0.1;
    ekf.setProcessNoiseCovariance(Q2);   
    EXPECT_NO_THROW(ekf.validate());
    EXPECT_EQ(ekf.getState().size(), 2);
    EXPECT_EQ(ekf.getLastEstimate().size(), 2);

    // check initialization of output
    Output out = ekf.getLastEstimate();
    OutputValue defaultOutVal;
    EXPECT_GT(out.size(), 0);
    EXPECT_DOUBLE_EQ(out.getValue(), defaultOutVal.getValue());

    // check setting the control input
    InputValue ctrl(0);
    Input in_ctrl(ctrl);
    EXPECT_THROW(ekf.setControlInput(in_ctrl), length_error);
    ekf.setControlInputSize(1);
    EXPECT_NO_THROW(ekf.setControlInput(in_ctrl));
  }

  TEST(ExtendedKalmanFilterTest, validation)
  {
    ExtendedKalmanFilter ekf;

    // check validation itself (e.g. invalid sizes of vectors/matrices,
    // empty callbacks, etc.)
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "STM missing"
    ekf.setStateTransitionModel(0);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "STM missing"
    ekf.setStateTransitionModel(f);
    ekf.setJacobianOfStateTransitionModel(0);	
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "STMJ missing"
    ekf.setJacobianOfStateTransitionModel(df);
    ekf.setObservationModel(0);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "OM missing"
    ekf.setObservationModel(h);
    ekf.setJacobianOfObservationModel(0);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "OMJ missing"
    ekf.setJacobianOfObservationModel(dh);

    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "PNC missing"
    MatrixXd Q_f(2,1); Q_f << 0.1, 1;
    ekf.setProcessNoiseCovariance(Q_f);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "Q not square"
    MatrixXd Q1(1,1); Q1 << 0.1;
    ekf.setProcessNoiseCovariance(Q1);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "MNC missing"
    MatrixXd R_f(2,1); R_f << 10, 1;
    ekf.setMeasurementNoiseCovariance(R_f);
    EXPECT_THROW(ekf.validate(), IEstimator::estimator_error);	// "R not square"
    MatrixXd R(1,1); R << 10;
    ekf.setMeasurementNoiseCovariance(R);

    EXPECT_NO_THROW(ekf.validate());				// all required params given

    // optional params
    VectorXd x(2); x << 0, 1;
    ekf.setInitialState(x);
    EXPECT_NO_THROW(ekf.validate());
    EXPECT_NE(ekf.getState().size(), 2);

    MatrixXd Q(2,2); Q << 0.1, 0, 0, 0.1;
    ekf.setProcessNoiseCovariance(Q);
    EXPECT_NO_THROW(ekf.validate());
    ekf.setInitialState(x);
    EXPECT_NO_THROW(ekf.validate());
    EXPECT_EQ(ekf.getState().size(), 2);			// sizes now ok

    // size of R cannot be checked, must match number of formulas in h
    // -> check during compile time
  }

  TEST(ExtendedKalmanFilterTest, validationEffect)
  {  
    ExtendedKalmanFilter ekf;

    ekf.setStateTransitionModel(f);
    ekf.setJacobianOfStateTransitionModel(df);
    ekf.setObservationModel(h);
    ekf.setJacobianOfObservationModel(dh);
    MatrixXd Q(1,1); Q << 0.1;
    ekf.setProcessNoiseCovariance(Q);
    MatrixXd R(1,1); R << 10;
    ekf.setMeasurementNoiseCovariance(R);
    ekf.setControlInputSize(1);		// initializes u, so it can be used in f

    // validate has an effect?
    InputValue measurement(1);
    Input in(measurement);
    Output out;
  
    EXPECT_THROW(out = ekf.estimate(in), IEstimator::estimator_error);	// "not yet validated"
  
    EXPECT_NO_THROW(ekf.validate());
    EXPECT_NO_THROW(out = ekf.estimate(in));

    // changing a parameter -> EKF must be re-validated
    ekf.setProcessNoiseCovariance(Q);
    EXPECT_THROW(out = ekf.estimate(in), IEstimator::estimator_error);	// "not yet validated"
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
    MatrixXd Q(1,1); Q << 0.1;
    ekf.setProcessNoiseCovariance(Q);
    MatrixXd R(1,1); R << 10;
    ekf.setMeasurementNoiseCovariance(R);

    ekf.validate();

    // check if the calculation of an estimate is correct
    // first measurement z1:
    InputValue measurement(1);
    Input in(measurement);
    Output out;

    // u contained in a state transition formula, so an error will be
    // thrown when u is not initialized with the correct size
    //out = ekf.estimate(in);		// fail assertion produced by Eigen (out-of-range error)
    ekf.setControlInputSize(1);
    ekf.validate();
    EXPECT_NO_THROW(out = ekf.estimate(in));

    EXPECT_EQ(out.size(), 1);
    EXPECT_NEAR(out[0].getValue(), 0.0099, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.0990, 0.0001);

    // next measurement z2:
    in[0].setValue(5);

    // setting a control input = 0 does not change the result, it can
    // be calculated as if there is no control input
    InputValue ctrl(0);
    Input in_ctrl(ctrl);
    ekf.setControlInput(in_ctrl);	// (no validation needed, cannot be validated)

    EXPECT_NO_THROW(out = ekf.estimate(in));
  
    EXPECT_NEAR(out[0].getValue(), 0.1073, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.1951, 0.0001);

    // another example
    ExtendedKalmanFilter ekf2;
    ekf2.setStateTransitionModel(f2);
    ekf2.setJacobianOfStateTransitionModel(df2);
    ekf2.setObservationModel(h2);
    ekf2.setJacobianOfObservationModel(dh2);
    MatrixXd Q2(2,2); Q2 << 0.1, 0, 0, 0.1;
    ekf2.setProcessNoiseCovariance(Q2);
    MatrixXd R2(1,1); R2 << 10;
    ekf2.setMeasurementNoiseCovariance(R2);

    ekf2.validate();

    in[0].setValue(1);
    EXPECT_NO_THROW(out = ekf2.estimate(in));
    EXPECT_EQ(out.size(), 2);
    
    EXPECT_NEAR(out[0].getValue(), 0.0, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.1, 0.0001);
    EXPECT_NEAR(out[1].getValue(), 0.0099, 0.0001);
    EXPECT_NEAR(out[1].getVariance(), 0.0990, 0.0001);
  }

}
