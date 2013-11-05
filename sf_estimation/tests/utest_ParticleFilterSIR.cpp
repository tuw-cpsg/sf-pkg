#include <gtest/gtest.h>
#include <stdexcept>
#include <Eigen/Dense>

// testing following API
#include "estimation/IEstimator.h"
#include "estimation/ParticleFilterSIR.h"
#include "estimation/Input.h"
#include "estimation/InputValue.h"
#include "estimation/Output.h"
#include "estimation/OutputValue.h"
#include "probability/sampling.h"

using namespace std;
using namespace estimation;
using namespace probability;

namespace ParticleFilterSIRTest 
{
  void f(VectorXd& x, const VectorXd& u)
  {
    x[0] = x[0] + u[0];
  }
  void h(VectorXd& z, const VectorXd& x)
  {
    z[0] = x[0];
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
  TEST(ParticleFilterSIRTest, initialization)
  {
    ParticleFilterSIR pf(100);

    // check setting required params ----------------------------
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "STM missing"
    pf.setStateTransitionModel(f);
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "OM missing"
    pf.setObservationModel(h);
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "PNC missing"
    MatrixXd Q(1,1); Q << 0.1;
    pf.setProcessNoiseCovariance(Q);
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "MNC missing"
    MatrixXd R(1,1); R << 10;
    pf.setMeasurementNoiseCovariance(R);

    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "particles not initialized"

    // check particle initializing ------------------------------
    ParticleFilterSIR pf_unititialized = pf;

    // init with known state
    VectorXd x0(1); x0 << -1;			// current state, i.e. mean
    pf.setInitialState(x0);			// init particles with mean
    EXPECT_NO_THROW(pf.validate());
    
    // init randomly
    pf = pf_unititialized;
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "particles not initialized"
    VectorXd a(1); a << -1;			// lower bound
    VectorXd b(1); b << 1;			// upper bound
    pf.initializeParticles(a, b);		// uniformly random
    EXPECT_NO_THROW(pf.validate());

    // check initialization of output ---------------------------
    // default output
    Output out = pf.getLastEstimate();
    OutputValue defaultOutVal;
    EXPECT_GT(out.size(), 0);
    EXPECT_DOUBLE_EQ(out.getValue(), defaultOutVal.getValue());

    // check setting the control input --------------------------
    InputValue ctrl(0);
    Input in_ctrl(ctrl);
    // when using control input, the length must be set before
    EXPECT_THROW(pf.setControlInput(in_ctrl), length_error);
    pf.setControlInputSize(1);
    EXPECT_NO_THROW(pf.setControlInput(in_ctrl));
  }

  TEST(ParticleFilterSIRTest, validation)
  {
    ParticleFilterSIR pf(100);

    // check validation itself (e.g. invalid sizes of vectors/matrices,
    // empty callbacks, etc.)
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "STM missing"
    pf.setStateTransitionModel(0);
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "STM missing"
    pf.setStateTransitionModel(f);
    pf.setObservationModel(0);
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "OM missing"
    pf.setObservationModel(h);
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "PNC missing"
    MatrixXd Q_f(2,2); Q_f << 1, 0.1, 0.1, 1;
    pf.setProcessNoiseCovariance(Q_f);
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "MNC missing"
    MatrixXd R_f(2,1); R_f << 10, 1;
    pf.setMeasurementNoiseCovariance(R_f);

    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "particles not initialized"
    VectorXd x0(1); x0 << -1;
    pf.setInitialState(x0);

    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "Q invalid size"
    MatrixXd Q(1,1); Q << 0.1;
    pf.setProcessNoiseCovariance(Q);
    EXPECT_THROW(pf.validate(), IEstimator::estimator_error);	// "R not square"
    MatrixXd R(1,1); R << 10;
    pf.setMeasurementNoiseCovariance(R);

    EXPECT_NO_THROW(pf.validate());				// all required params given

    // size of R cannot be checked, must match number of formulas in h
    // -> check during compile time
  }

  TEST(ParticleFilterSIRTest, validationEffect)
  {  
    ParticleFilterSIR pf(100);

    // a correct initialization
    pf.setStateTransitionModel(f);
    pf.setObservationModel(h);
    MatrixXd Q(1,1); Q << 0.1;
    pf.setProcessNoiseCovariance(Q);
    MatrixXd R(1,1); R << 10;
    pf.setMeasurementNoiseCovariance(R);
    VectorXd x0(1); x0 << -3;
    pf.setInitialState(x0);
    pf.setControlInputSize(1);		// initializes u, so it can be used in f

    // validate has an effect? ----------------------------------
    InputValue measurement(-3);
    Input in(measurement);
    Output out;
  
    EXPECT_THROW(out = pf.estimate(in), IEstimator::estimator_error);	// "not yet validated"
    EXPECT_NO_THROW(pf.validate());
    EXPECT_NO_THROW(out = pf.estimate(in));

    // changing a parameter -> PF must be re-validated
    pf.setProcessNoiseCovariance(Q);
    EXPECT_THROW(out = pf.estimate(in), IEstimator::estimator_error);	// "not yet validated"
    EXPECT_NO_THROW(pf.validate());
  
    // PF should now be released and return an estimate
    EXPECT_NO_THROW(out = pf.estimate(in));
    EXPECT_GT(out.size(), 0);
    OutputValue defaultOutVal;
    // estimated value, not the default
    EXPECT_NE(out.getValue(), defaultOutVal.getValue());
  }

  TEST(ParticleFilterSIRTest, functionality)
  {
    ParticleFilterSIR pf(1000);

    pf.setStateTransitionModel(f);
    pf.setObservationModel(h);
    MatrixXd Q(1,1); Q << 0.1;
    pf.setProcessNoiseCovariance(Q);
    MatrixXd R(1,1); R << 10;
    pf.setMeasurementNoiseCovariance(R);
    VectorXd x0(1); x0 << -3;
    pf.setInitialState(x0);

    pf.validate();

    // check if the calculation of an estimate is correct
    // first measurement z1:
    InputValue measurement(-3);		// should result in the same value with higher variance
    Input in(measurement);
    Output out;

    // u contained in a state transition formula, so an error will be
    // thrown when u is not initialized with the correct size
    pf.setControlInputSize(1);			// sets default control input = 0
    pf.validate();				// must be validated
    EXPECT_NO_THROW(out = pf.estimate(in));	// estimation should work

    EXPECT_EQ(out.size(), 1);
    EXPECT_NEAR(out[0].getValue(), -3, 0.5);
    EXPECT_NE(out[0].getVariance(), 0);		// variance should have changed
    EXPECT_NEAR(out[0].getVariance(), 0, 0.5);

    // control input = 0 should not change values
    InputValue ctrlVal(0);
    Input ctrl(ctrlVal);
    pf.setControlInput(ctrl);
    out = pf.estimate(in);
    EXPECT_NEAR(out[0].getValue(), -3, 0.5);

    double var;
    var = out[0].getVariance();

    // missing measurement
    InputValue missingValue;
    Input inMissing(missingValue);
    EXPECT_NO_THROW(out = pf.estimate(inMissing));
  
    EXPECT_NEAR(out[0].getValue(), -3, 0.5);	// only time update
    EXPECT_GT(out[0].getVariance(), var);	// variance increased

    // multi dimensional example -------------------------------------
    ParticleFilterSIR pf2(1000);
    
    pf.setStateTransitionModel(f2);
    pf.setObservationModel(h2);
    MatrixXd Q2(2,2); Q2 << 0.1, 0.01, 0.01, 0.1;
    pf.setProcessNoiseCovariance(Q2);
    MatrixXd R2(1,1); R2 << 10;
    pf.setMeasurementNoiseCovariance(R2);
    VectorXd x02(2); x02 << -3, 3;
    pf.setInitialState(x02);

    pf.validate();
    EXPECT_NO_THROW(out = pf.estimate(in));	// estimation should work

    ASSERT_EQ(out.size(), 2);
    EXPECT_NEAR(out[0].getValue(), -3, 0.5);
    EXPECT_NEAR(out[1].getValue(), 3, 1);
  }
}
