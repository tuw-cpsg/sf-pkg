#include <gtest/gtest.h>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>

// testing following API
#include "estimation/IEstimator.h"
#include "estimation/KalmanFilter.h"
#include "estimation/Input.h"
#include "estimation/InputValue.h"
#include "estimation/Output.h"
#include "estimation/OutputValue.h"

using namespace std;
using namespace estimation;

namespace KalmanFilterTest 
{
  // -----------------------------------------
  // tests
  // -----------------------------------------
  TEST(KalmanFilterTest, initializationAndValidation)
  {
    KalmanFilter kf;

    // check setting required params
    EXPECT_THROW(kf.validate(), IEstimator::estimator_error);	// "STM missing"
    vector< vector<double> > A_f = { {1,0} };
    kf.setStateTransitionModel(A_f);
    EXPECT_THROW(kf.validate(), IEstimator::estimator_error);	// "OM missing"
    vector< vector<double> > H = { {1,0} };
    kf.setObservationModel(H);
    EXPECT_THROW(kf.validate(), IEstimator::estimator_error);	// "PNC missing"
    vector< vector<double> > Q = { {0.1,0},{0,0.1} };
    kf.setProcessNoiseCovariance(Q);
    EXPECT_THROW(kf.validate(), IEstimator::estimator_error);	// "MNC missing"
    vector< vector<double> > R = { {10} };
    kf.setMeasurementNoiseCovariance(R);

    EXPECT_THROW(kf.validate(), IEstimator::estimator_error);	// "STM invalid size"
    vector< vector<double> > A = { {1,0},{0,1} };
    kf.setStateTransitionModel(A);
    EXPECT_NO_THROW(kf.validate());			// all required params given

    vector< vector<double> > R_f = { {10,1} };
    kf.setMeasurementNoiseCovariance(R_f);
    EXPECT_THROW(kf.validate(), IEstimator::estimator_error);	// "MNC invalid size"
    kf.setMeasurementNoiseCovariance(R);
    vector< vector<double> > H_f = { {1,0,2} };
    kf.setObservationModel(H_f);
    EXPECT_THROW(kf.validate(), IEstimator::estimator_error);	// "OM invalid size"
    kf.setObservationModel(H);

    // check setting optional params
    /*
    // check only for first initialization/validation
    vector<double> u = { 0 };
    kf.setControlInput(u);
    EXPECT_THROW(kf.validate(), IEstimator::estimator_error);	// "CIM missing"
    vector< vector<double> > B_f = { {0} };
    kf.setControlInputModel(B_f);
    EXPECT_THROW(kf.validate(), IEstimator::estimator_error);	// "CIM invalid size"
    vector< vector<double> > B = { {0},{0} };
    kf.setControlInputModel(B);
    EXPECT_NO_THROW(kf.validate());
    */
    vector<double> x = { 0, 1 };
    kf.setInitialState(x);
    EXPECT_NO_THROW(kf.validate());

    vector< vector<double> > P0_f = { {1},{0} };
    kf.setInitialErrorCovariance(P0_f);
    EXPECT_THROW(kf.validate(), IEstimator::estimator_error);	// "P0 invalid size"
    vector< vector<double> > P0_f1 = { {1,0} };
    kf.setInitialErrorCovariance(P0_f1);
    EXPECT_NO_THROW(kf.validate());				// "P0 invalid but will be set to 0"
  
    // check initialization of output
    Output out = kf.getLastEstimate();
    OutputValue defaultOutVal;
    EXPECT_GT(out.size(), 0);
    EXPECT_DOUBLE_EQ(out.getValue(), defaultOutVal.getValue());
  }

  TEST(KalmanFilterTest, validationEffect)
  {  
    KalmanFilter kf;

    vector< vector<double> > A = { {1} };
    kf.setStateTransitionModel(A);
    vector< vector<double> > H = { {1} };
    kf.setObservationModel(H);
    vector< vector<double> > Q = { {0.1} };
    kf.setProcessNoiseCovariance(Q);
    vector< vector<double> > R = { {10} };
    kf.setMeasurementNoiseCovariance(R);

    // validate has an effect?
    InputValue measurement(1);
    Input in(measurement);
    Output out;
  
    EXPECT_THROW(out = kf.estimate(in), IEstimator::estimator_error);	// "not yet validated"
  
    EXPECT_NO_THROW(kf.validate());

    // changing a parameter -> KF must be re-validated
    kf.setProcessNoiseCovariance(Q);
    EXPECT_THROW(out = kf.estimate(in), IEstimator::estimator_error);	// "not yet validated"
    EXPECT_NO_THROW(kf.validate());
  
    // KF should now be released and return an estimate (should not be
    // the default)
    EXPECT_NO_THROW(out = kf.estimate(in));
    EXPECT_GT(out.size(), 0);
    EXPECT_EQ(kf.getState().size(), out.size());
  }

  TEST(KalmanFilterTest, functionality)
  {
    KalmanFilter kf;

    vector< vector<double> > A = { {1} };
    kf.setStateTransitionModel(A);
    vector< vector<double> > H = { {1} };
    kf.setObservationModel(H);
    vector< vector<double> > Q = { {0.1} };
    kf.setProcessNoiseCovariance(Q);
    vector< vector<double> > R = { {10} };
    kf.setMeasurementNoiseCovariance(R);

    kf.validate();
    
    // check if the calculation of an estimate is correct
    InputValue measurement(1);
    Input in(measurement);
    Output out = kf.estimate(in);
  
    EXPECT_EQ(out.size(), 1);
    EXPECT_NEAR(out[0].getValue(), 0.0099, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.0990, 0.0001);

    in[0].setValue(5);
    EXPECT_NO_THROW(out = kf.estimate(in));
  
    EXPECT_NEAR(out[0].getValue(), 0.1073, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.1951, 0.0001);

    // another example
    KalmanFilter kf2;
    vector< vector<double> > A2 = { {1,0.1},{0,1} };
    kf2.setStateTransitionModel(A2);
    vector< vector<double> > H2 = { {0,1} };
    kf2.setObservationModel(H2);
    vector< vector<double> > Q2 = { {0.1,0},{0,0.1} };
    kf2.setProcessNoiseCovariance(Q2);
    vector< vector<double> > R2 = { {10} };
    kf2.setMeasurementNoiseCovariance(R2);

    kf2.validate();

    in[0].setValue(1);
    EXPECT_NO_THROW(out = kf2.estimate(in));
    EXPECT_EQ(out.size(), 2);
    
    EXPECT_NEAR(out[0].getValue(), 0.0, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.1, 0.0001);
    EXPECT_NEAR(out[1].getValue(), 0.0099, 0.0001);
    EXPECT_NEAR(out[1].getVariance(), 0.0990, 0.0001);
  }

}
