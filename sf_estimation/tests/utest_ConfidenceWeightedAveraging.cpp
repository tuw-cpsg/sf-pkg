#include <gtest/gtest.h>
#include <stdexcept>

// testing following API
#include "estimation/IEstimator.h"
#include "estimation/ConfidenceWeightedAveraging.h"
#include "estimation/Input.h"
#include "estimation/InputValue.h"
#include "estimation/Output.h"
#include "estimation/OutputValue.h"

using namespace std;
using namespace estimation;

namespace ConfidenceWeightedAveragingTest 
{
  // -----------------------------------------
  // tests
  // -----------------------------------------
  TEST(ConfidenceWeightedAveragingTest, initialization)
  {
    ConfidenceWeightedAveraging cwa;

    // check initialization of output ---------------------------
    // default output
    Output out = cwa.getLastEstimate();
    OutputValue defaultOutVal;
    EXPECT_EQ(out.size(), 1);
    EXPECT_DOUBLE_EQ(out[0].getValue(), defaultOutVal.getValue());
    EXPECT_DOUBLE_EQ(out[0].getVariance(), defaultOutVal.getVariance());
  }

  TEST(ConfidenceWeightedAveragingTest, functionality)
  {
    ConfidenceWeightedAveraging cwa;

    // test input values
    InputValue x1(1);
    InputValue x2(0);
    InputValue x3(-1);
    InputValue missingValue;
    Input in;
    in.add(x1);
    in.add(x2);
    in.add(x3);
    Output out;

    // all zero variance -> simple averaging
    cwa.setIgnoreZeroVarianceValues();

    EXPECT_NO_THROW(out = cwa.estimate(in));	// valid inputs
    EXPECT_DOUBLE_EQ(out[0].getValue(), 0);
    EXPECT_NEAR(out[0].getVariance(), 0.6667, 0.0001);
    
    cwa.clearIgnoreZeroVarianceValues();

    // do not ignore zero variance values -----------------------
    EXPECT_ANY_THROW(out = cwa.estimate(in));	// zero variance input
    in[0].setVariance(0.1);
    in[1].setVariance(0.4);
    EXPECT_ANY_THROW(out = cwa.estimate(in));	// zero variance input
    in[2].setVariance(0.5);
    EXPECT_NO_THROW(out = cwa.estimate(in));	// valid inputs
    
    EXPECT_NEAR(out[0].getValue(), 0.55172, 0.00001);
    EXPECT_NEAR(out[0].getVariance(), 0.06897, 0.00001);

    // ignore zero variance values ------------------------------
    in[0].setVariance(0);
    cwa.setIgnoreZeroVarianceValues();

    EXPECT_NO_THROW(out = cwa.estimate(in));	// valid inputs: in[1] (0,0.4), in[2] (-1,0.5)
    EXPECT_NEAR(out[0].getValue(), -0.4444, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.2222, 0.0001);

    // missing measurement --------------------------------------
    in[0] = missingValue;

    EXPECT_NO_THROW(out = cwa.estimate(in));
    EXPECT_NEAR(out[0].getValue(), -0.4444, 0.0001);
    EXPECT_NEAR(out[0].getVariance(), 0.2222, 0.0001);

    // zero variance and missing --------------------------------
    in[0] = missingValue;
    in[1].setVariance(0);
    
    EXPECT_NO_THROW(out = cwa.estimate(in));
    EXPECT_DOUBLE_EQ(out[0].getValue(), -1);
    EXPECT_DOUBLE_EQ(out[0].getVariance(), 0.5);    

    // all measurements missing ---------------------------------
    in[0] = missingValue; 
    in[1] = missingValue;
    in[2] = missingValue;

    EXPECT_ANY_THROW(out = cwa.estimate(in));
  }
}
