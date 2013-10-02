#include <gtest/gtest.h>
#include <typeinfo>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

// testing following API
#include "estimation/EstimatorFactory.h"

using namespace std;
using namespace estimation;

// -----------------------------------------
// tests
// -----------------------------------------
TEST(EstimatorFactoryTest, addParams)
{
  EstimatorFactory cfg;

  EXPECT_NO_THROW(cfg.addParam("string", string("xyz")));
  EXPECT_NO_THROW(cfg.addParam("int", 5));
  VectorXd v(5); v << 1, 0, 1, 3, 1;
  EXPECT_NO_THROW(cfg.addParam("vector", v));
  MatrixXd m(2,2); m << 1,0,0,1;
  EXPECT_NO_THROW(cfg.addParam("matrix", m));
  EXPECT_NO_THROW(cfg.addParam("matrix", v));
}

// add params and create according estimator -------------------

TEST(EstimatorFactoryTest, invalidMethods)
{
  EstimatorFactory cfg;
  estimation::IEstimator* estimator;
  string classname;

  // no params, invalid
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error);
  cfg.addParam("method", string("MovingMedian"));

  // check if reset works
  cfg.reset();
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error);

  // invalid methods
  cfg.addParam("method", 5);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error);
  cfg.addParam("method", string("xy"));
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error);
}

TEST(EstimatorFactoryTest, movingMedian)
{
  EstimatorFactory cfg;
  estimation::IEstimator* estimator;
  string classname;

  cfg.addParam("method", string("MovingMedian"));
  EXPECT_NO_THROW(estimator = cfg.create());
  classname = typeid(*estimator).name();
  EXPECT_NE(string::npos, classname.find("MovingMedian",0));

  // optional params
  cfg.addParam("window-size", 5);
  EXPECT_NO_THROW(estimator = cfg.create());
  classname = typeid(*estimator).name();
  EXPECT_NE(string::npos, classname.find("MovingMedian",0));
}

TEST(EstimatorFactoryTest, movingAverage)
{
  EstimatorFactory cfg;
  estimation::IEstimator* estimator;
  string classname;

  cfg.addParam("method", string("MovingAverage"));
  EXPECT_NO_THROW(estimator = cfg.create());

  // optional params
  cfg.addParam("window-size", 5);
  VectorXd wc1(4); wc1 << 1, 2, 5, 2;
  cfg.addParam("weighting-coefficients", wc1);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "invalid weighting coefficients"

  VectorXd wc2(5); wc2 << 1, 2, 5, 2, 1;
  cfg.addParam("weighting-coefficients", wc2);
  EXPECT_NO_THROW(estimator = cfg.create());
  classname = typeid(*estimator).name();
  EXPECT_NE(string::npos, classname.find("MovingAverage",0));
}

TEST(EstimatorFactoryTest, kalmanFilter)
{
  EstimatorFactory cfg;
  estimation::IEstimator* estimator;
  string classname;

  // check a kalman filter
  cfg.addParam("method", string("KalmanFilter"));

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "STM missing"
  VectorXd stm_f(1); stm_f << 1;
  cfg.addParam("state-transition-model", stm_f);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  MatrixXd stm(1,1); stm << 1;
  cfg.addParam("state-transition-model", stm);

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "PNC missing"
  int pnc_f = 1;
  cfg.addParam("process-noise-covariance", pnc_f);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  MatrixXd pnc(1,1); pnc << 0.2;
  cfg.addParam("process-noise-covariance", pnc);

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "OM missing"
  std::string om_f = "xy";
  cfg.addParam("observation-model", om_f);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  MatrixXd om(1,1); om << 1;
  cfg.addParam("observation-model", om);

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "MNC missing"
  double mnc_f = 0.1;
  cfg.addParam("measurement-noise-covariance", mnc_f);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  MatrixXd mnc(1,1); mnc << 10;
  cfg.addParam("measurement-noise-covariance", mnc);

  EXPECT_NO_THROW(estimator = cfg.create());
  classname = typeid(*estimator).name();
  EXPECT_NE(string::npos, classname.find("KalmanFilter",0)); // found KalmanFilter in classname!

  // optional params
  cfg.addParam("control-input-model", stm_f);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  MatrixXd cim(1,1); cim << 1;
  cfg.addParam("control-input-model", cim);
  EXPECT_NO_THROW(estimator = cfg.create());

  cfg.addParam("control-input", pnc);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  VectorXd ci(1); ci << 0;
  cfg.addParam("control-input", ci);
  EXPECT_NO_THROW(estimator = cfg.create());

  cfg.addParam("initial-state", stm);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  VectorXd is(1); is << -1.5;
  cfg.addParam("initial-state", is);
  EXPECT_NO_THROW(estimator = cfg.create());

  cfg.addParam("initial-error-covariance", is);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  MatrixXd iec(1,1); iec << 1;
  cfg.addParam("initial-error-covariance", iec);
  EXPECT_NO_THROW(estimator = cfg.create());
}

void test_f(VectorXd& x, const VectorXd& u) { }
void test_df(MatrixXd& A, const VectorXd& x, const VectorXd& u) { }
void test_h(VectorXd& z, const VectorXd& x) { }
void test_dh(MatrixXd& H, const VectorXd& x) { }

TEST(EstimatorFactoryTest, extendedKalmanFilter)
{
  EstimatorFactory cfg;
  estimation::IEstimator* estimator;
  string classname;

  // check required params
  cfg.addParam("method", string("ExtendedKalmanFilter"));

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "STM missing"
  VectorXd stm_f(1); stm_f << 1;
  cfg.addParam("state-transition-model", stm_f);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  ExtendedKalmanFilter::func_f stm = 0;
  cfg.addParam("state-transition-model", stm);

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "STMJ missing"
  int stmj_f = 1;
  cfg.addParam("state-transition-model-jacobian", stmj_f);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  ExtendedKalmanFilter::func_df stmj = test_df;
  cfg.addParam("state-transition-model-jacobian", stmj);

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "PNC missing"
  int pnc_f = 1;
  cfg.addParam("process-noise-covariance", pnc_f);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  MatrixXd pnc(1,1); pnc << 0.2;
  cfg.addParam("process-noise-covariance", pnc);

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "OM missing"
  cfg.addParam("observation-model", stm);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  ExtendedKalmanFilter::func_h om = test_h;
  cfg.addParam("observation-model", om);

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "OMJ missing"
  cfg.addParam("observation-model-jacobian", stm);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  ExtendedKalmanFilter::func_dh omj = test_dh;
  cfg.addParam("observation-model-jacobian", omj);

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "MNC missing"
  double mnc_f = 0.1;
  cfg.addParam("measurement-noise-covariance", mnc_f);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  MatrixXd mnc(1,1); mnc << 10;
  cfg.addParam("measurement-noise-covariance", mnc);

  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "validation failed (f=0)"
  stm = test_f;
  cfg.addParam("state-transition-model", stm);

  EXPECT_NO_THROW(estimator = cfg.create());				   // all required params passed


  // optional params
  cfg.addParam("control-input", pnc);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  VectorXd ci(1); ci << 0;
  cfg.addParam("control-input", ci);
  EXPECT_NO_THROW(estimator = cfg.create());

  cfg.addParam("initial-state", stm);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  VectorXd is(1); is << -1.5;
  cfg.addParam("initial-state", is);
  EXPECT_NO_THROW(estimator = cfg.create());

  cfg.addParam("initial-error-covariance", is);
  EXPECT_THROW(estimator = cfg.create(), EstimatorFactory::factory_error); // "bad cast"
  MatrixXd iec(1,1); iec << 1;
  cfg.addParam("initial-error-covariance", iec);
  EXPECT_NO_THROW(estimator = cfg.create());

  // validation is done in utest_ExtendedKalmanFilter?

  EXPECT_NO_THROW(estimator = cfg.create());
  classname = typeid(*estimator).name();
  EXPECT_NE(string::npos, classname.find("ExtendedKalmanFilter",0)); // found ExtendedKalmanFilter in classname!
}

