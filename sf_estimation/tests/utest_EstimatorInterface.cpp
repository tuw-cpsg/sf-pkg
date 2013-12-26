#include <gtest/gtest.h>
#include <stdexcept>
#include <Eigen/Dense>
#include <ctime>

// testing following API
#include "estimation/Output.h"
#include "estimation/OutputValue.h"

using namespace std;
using namespace estimation;

#define CLOCK_RESOLUTION	10

namespace EstimatorInterfaceTest 
{
  // -----------------------------------------
  // tests
  // -----------------------------------------
  TEST(EstimatorInterfaceTest, outputInitialization)
  {
    OutputValue empty;
    Output outSingle(empty);

    OutputValue x(-1,0,1);
    OutputValue y(11,2,20);
    vector<OutputValue> vals;
    vals.push_back(x);
    Output outMulti(vals);
    outMulti.add(y);
    
    EXPECT_EQ(outSingle.size(), 1);
    EXPECT_EQ(outMulti.size(), 2);
    
    outMulti.clear();

    EXPECT_EQ(outMulti.size(), 0);
  }

  TEST(EstimatorInterfaceTest, outputOperationsOn)
  {
    OutputValue empty;
    Output out(empty), temp, outMulti;
    
    // assignment
    temp = out;

    EXPECT_EQ(temp.size(), out.size());
    EXPECT_EQ(temp[0], out[0]);

    // [], fill with many values
    int N = 100;
    for (int i = 0; i < N; i++)
    {
      outMulti.add(empty);
      
      // check defaults
      EXPECT_DOUBLE_EQ(outMulti[i].getValue(), -1);
      EXPECT_DOUBLE_EQ(outMulti[i].getVariance(), 0);
      EXPECT_NEAR(outMulti[i].getJitter(), 0, CLOCK_RESOLUTION + 0.01);
    }

    EXPECT_ANY_THROW(outMulti[N].getValue());
    
    // [], set values
    for (int i = 0; i < outMulti.size(); i++)
    {
      outMulti[i].setValue(i + 100);
      outMulti[i].setVariance(i + 10);

      EXPECT_DOUBLE_EQ(outMulti[i].getValue(), i + 100);
      EXPECT_DOUBLE_EQ(outMulti[i].getVariance(), i + 10);
      EXPECT_NEAR(outMulti[i].getJitter(), 0, CLOCK_RESOLUTION + 0.01);

      outMulti[i].setJitter(i*20);
      
      EXPECT_NEAR(outMulti[i].getJitter(), i*20, CLOCK_RESOLUTION + 0.01);
    }
  }
}
