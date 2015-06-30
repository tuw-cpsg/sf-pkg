#include <gtest/gtest.h>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include <cmath>

// testing following API
#include "estimation/UnscentedKalmanFilter.h"
#include "probability/sampling.h"

using namespace std;
using namespace estimation;
using namespace probability;

namespace UnscentedKalmanFilterTest 
{
  void f(VectorXd& x, const VectorXd& u)
  {
    x[0] = x[0] + u[0];
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

  // 
  // Simple nonlinear system: a robot drives with constant velocity
  // and constant heading. Distance made per cycle and heading are
  // measured directly. The measurements are generated with the
  // sampling functions available.
  // 
  void fn(VectorXd& x, const VectorXd& u)
  {
    VectorXd x_apriori(x.size());

    x_apriori[0] = x[0] + x[1]*cos(x[2]);	// x
    x_apriori[1] = x[1];			// ds
    x_apriori[2] = x[2];			// th
    
    x = x_apriori;    
  }

  void hn(VectorXd& z, const VectorXd& x)
  {
    z[0] = x[1];	// ds
    z[1] = x[2];	// th
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
    MatrixXd Q(1,1); Q << 0.1;
    ukf.setProcessNoiseCovariance(Q);
    EXPECT_THROW(ukf.validate(), IEstimator::estimator_error);	// "MNC missing"
    MatrixXd R(1,1); R << 10;
    ukf.setMeasurementNoiseCovariance(R);
    EXPECT_NO_THROW(ukf.validate());			// all required params given

    // check setting optional params
    // optional params doesn't set new sizes
    MatrixXd P0_f(2,2); P0_f << 1, 0, 0, 1;
    ukf.setInitialErrorCovariance(P0_f);
    EXPECT_NO_THROW(ukf.validate());		// validate resets P0 to zero-matrix of correct size
    EXPECT_EQ(ukf.getState().size(), 1);	// correct size = 1 (see Q)

    VectorXd x_f(2); x_f << 0,1;
    ukf.setInitialState(x_f);
    EXPECT_NO_THROW(ukf.validate());		// validate resets x0 to zero-vector of correct size
    EXPECT_NE(ukf.getState().size(), 2);	// correct size = 1 (see Q)

    // required param Q sets new size of x and out
    MatrixXd Q2(2,2); Q2 << 0.1, 0, 0, 0.1;
    ukf.setProcessNoiseCovariance(Q2);   
    EXPECT_NO_THROW(ukf.validate());
    EXPECT_EQ(ukf.getState().size(), 2);
    EXPECT_EQ(ukf.getLastEstimate().size(), 2);
    
    // check initialization of output
    Output out = ukf.getLastEstimate();
    OutputValue defaultOutVal;
    EXPECT_GT(out.size(), 0);
    EXPECT_DOUBLE_EQ(out.getValue(), defaultOutVal.getValue());

    // check setting the control input
    InputValue ctrl(0);
    Input in_ctrl(ctrl);
    EXPECT_THROW(ukf.setControlInput(in_ctrl), length_error);
    ukf.setControlInputSize(1);
    EXPECT_NO_THROW(ukf.setControlInput(in_ctrl));
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
    MatrixXd Q(1,1); Q << 0.1;
    ukf.setProcessNoiseCovariance(Q);
    MatrixXd R(1,1); R << 10;
    ukf.setMeasurementNoiseCovariance(R);

    ukf.validate();

    // check if the calculation of an estimate is correct
    InputValue measurement(1);
    Input in(measurement);
    Output out;

    // u contained in a state transition formula, so an error will be
    // thrown when u is not initialized with the correct size
    //out = ekf.estimate(in);		// fail assertion produced by Eigen (out-of-range error)
    ukf.setControlInputSize(1);
    ukf.validate();

    // setting a control input = 0 does not change the result, it can
    // be calculated as if there is no control input
    InputValue ctrl(0);
    Input in_ctrl(ctrl);
    ukf.setControlInput(in_ctrl);	// (no validation needed, cannot be validated)

    EXPECT_NO_THROW(out = ukf.estimate(in));

    EXPECT_EQ(out.size(), 1);
    EXPECT_NEAR(out[0].getValue(), 0.0099, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.0990, 0.0001);

    // missing measurement
    InputValue missingValue;
    Input inMissing(missingValue);
    EXPECT_NO_THROW(out = ukf.estimate(inMissing));
  
    EXPECT_NEAR(out[0].getValue(), 0.0099, 0.0001);
  }

  TEST(UnscentedKalmanFilterTest, functionalityNonlinear)
  {
    UnscentedKalmanFilter ukf;
    const double heading = 80*M_PI/180, distance = 1;
    
    ukf.setStateTransitionModel(fn);
    ukf.setObservationModel(hn);
    MatrixXd Q(3,3); 
    Q << 0, 0, 0, 
      0, 0.01, 0,
      0, 0, 0.01;
    ukf.setProcessNoiseCovariance(Q);
    ukf.setInitialErrorCovariance(Q);
    MatrixXd R(2,2); 
    R << 0.05, 0,
      0, 0.03;
    ukf.setMeasurementNoiseCovariance(R);
    VectorXd x0(3);
    x0 << 0, distance, heading;
    ukf.setInitialState(x0);
    ukf.validate();

    // estimate
    VectorXd z(2);
    VectorXd zMean(2); zMean << distance, heading;
    MatrixXd zCov(2,2); zCov << 0.05, 0, 0, 0.03;

    InputValue ds(distance);
    InputValue th(heading);
    Input in; 
    in.add(ds); 
    in.add(th);
    Output out;

    // estimate with exact values
    out = ukf.estimate(in);
    EXPECT_NEAR(out[0].getValue(), distance*cos(heading), 0.1);
    EXPECT_NEAR(out[1].getValue(), distance, 0.05);
    EXPECT_NEAR(out[2].getValue(), heading, 0.03);

    // estimate with sample measurements
    ukf.setInitialState(x0);
    ukf.validate();

    for (int i = 0; i < 20; i++)
    {
      // sample measurements
      z = sampleNormalDistribution(zMean, zCov);
      in[0].setValue(z[0]);
      in[1].setValue(z[1]);

      out = ukf.estimate(in);
    }

    EXPECT_NEAR(out[0].getValue(), 20*(distance*cos(heading)), 1);
    EXPECT_NEAR(out[1].getValue(), distance, 0.5);
    EXPECT_NEAR(out[2].getValue(), heading, 0.3);
  }
}
