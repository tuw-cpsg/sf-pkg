#include <gtest/gtest.h>
#include <stdexcept>
#include <Eigen/Dense>
#include <ctime>

#include <iostream>

// testing following API
#include "estimation/OutputValue.h"

using namespace std;
using namespace estimation;

#define CLOCK_RESOLUTION	10

namespace OutputValueTest 
{
  // -----------------------------------------
  // tests
  // -----------------------------------------
  TEST(OutputValueTest, initialization)
  {
    bool incorrect = false;
    clock_t last;

    // check default value
    for (int i = 0; i < 1000; i++)
    {
      OutputValue empty;
      last = clock();

      EXPECT_DOUBLE_EQ(empty.getValue(), -1);
      EXPECT_DOUBLE_EQ(empty.getVariance(), 0);
      EXPECT_NEAR(empty.getJitter(), 0, CLOCK_RESOLUTION + 0.01);
    }

    OutputValue val(1,2,3);
    clock_t start_time = clock();

    EXPECT_DOUBLE_EQ(val.getValue(), 1);
    EXPECT_DOUBLE_EQ(val.getVariance(), 2);
    EXPECT_NEAR(val.getJitter(), 3, CLOCK_RESOLUTION + 0.01);
    
    // wait a little bit
    for (int i = 0; i < 10000; i++)
      for (int j = 0; j < 10000; j++);

    clock_t end_time = clock();
    double elapsed_time_ms = (double)(end_time - start_time)*1000/CLOCKS_PER_SEC;
    
    EXPECT_NEAR(val.getJitter(), 3+elapsed_time_ms, CLOCK_RESOLUTION + 0.01);

    // test setters
    OutputValue fill;
    double value = 0;
    for (int i = 0; i < 1000; i++)
    {
      value += 0.1;

      fill.setValue(value);
      fill.setVariance(0.1);

      EXPECT_DOUBLE_EQ(fill.getValue(), value);
      EXPECT_DOUBLE_EQ(fill.getVariance(), 0.1);
      EXPECT_NEAR(fill.getJitter(), 0, CLOCK_RESOLUTION + 0.01);

      fill.setJitter(20);
      
      EXPECT_NEAR(fill.getJitter(), 20, CLOCK_RESOLUTION + 0.01);
    }
  }

  TEST(OutputValueTest, operationsOn)
  {
    OutputValue x(-9,1,2);
    OutputValue y(-1,0,1);
    OutputValue temp;

    // equality
    temp = x;
    EXPECT_DOUBLE_EQ(temp.getValue(), x.getValue());
    EXPECT_DOUBLE_EQ(temp.getVariance(), x.getVariance());
    EXPECT_NEAR(temp.getJitter(), x.getJitter(), CLOCK_RESOLUTION + 0.01);

    EXPECT_TRUE(temp == x);
    EXPECT_FALSE(temp == y);
    EXPECT_TRUE(temp != y);
    
    // <,>
    EXPECT_TRUE(x < y);
    EXPECT_TRUE(y > x);
    EXPECT_FALSE(x < temp);
    EXPECT_FALSE(y < x);

    // <=,>=
    EXPECT_TRUE(temp <= x);
    EXPECT_TRUE(x <= temp);
    EXPECT_TRUE(temp >= x);
    EXPECT_TRUE(x >= temp);
    EXPECT_TRUE(x <= y);
    EXPECT_FALSE(x >= y);
  }
}
