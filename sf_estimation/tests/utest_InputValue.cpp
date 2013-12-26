#include <gtest/gtest.h>
#include <stdexcept>
#include <ctime>

#include <iostream>

// testing following API
#include "estimation/InputValue.h"

using namespace std;
using namespace estimation;

#define CLOCK_RESOLUTION	10

namespace InputValueTest 
{
  // -----------------------------------------
  // tests
  // -----------------------------------------
  TEST(InputValueTest, initialization)
  {
    bool incorrect = false;
    clock_t last;

    // check default value
    InputValue empty;
    last = clock();

    EXPECT_DOUBLE_EQ(empty.getValue(), -1);
    EXPECT_DOUBLE_EQ(empty.getVariance(), 0);
    EXPECT_NEAR(empty.getJitter(), 0, CLOCK_RESOLUTION + 0.01);
    EXPECT_EQ(empty.isMissing(), true);

    // check other constructors
    InputValue val(1);
    EXPECT_DOUBLE_EQ(val.getValue(), 1);
    EXPECT_DOUBLE_EQ(val.getVariance(), 0);
    EXPECT_NEAR(val.getJitter(), 0, CLOCK_RESOLUTION + 0.01);
    EXPECT_EQ(val.isMissing(), false);

    InputValue all(1,2,3);
    clock_t start_time = clock();

    EXPECT_DOUBLE_EQ(all.getValue(), 1);
    EXPECT_DOUBLE_EQ(all.getVariance(), 2);
    EXPECT_NEAR(all.getJitter(), 3, CLOCK_RESOLUTION + 0.01);
    EXPECT_EQ(all.isMissing(), false);
    
    // wait a little bit
    for (int i = 0; i < 10000; i++)
      for (int j = 0; j < 10000; j++);

    clock_t end_time = clock();
    double elapsed_time_ms = (double)(end_time - start_time)*1000/CLOCKS_PER_SEC;
    
    EXPECT_NEAR(val.getJitter(), 3+elapsed_time_ms, CLOCK_RESOLUTION + 0.01);

    // test value setter (missing is an additional member, rest is
    // checked in OutputValueTest)
    InputValue fill;

    EXPECT_EQ(fill.isMissing(), true);
    fill.setValue(-1);
    EXPECT_EQ(fill.isMissing(), false);
    fill.setVariance(0.1);

    EXPECT_DOUBLE_EQ(fill.getValue(), -1);
    EXPECT_DOUBLE_EQ(fill.getVariance(), 0.1);
    EXPECT_NEAR(fill.getJitter(), 0, CLOCK_RESOLUTION + 0.01);

    fill.setJitter(20);
      
    EXPECT_NEAR(fill.getJitter(), 20, CLOCK_RESOLUTION + 0.01);
  }

  TEST(InputValueTest, operationsOn)
  {
    InputValue x(-9,1,2);
    InputValue y(-1,0,1);
    InputValue temp;

    // equality
    temp = x;
    EXPECT_DOUBLE_EQ(temp.getValue(), x.getValue());
    EXPECT_DOUBLE_EQ(temp.getVariance(), x.getVariance());
    EXPECT_NEAR(temp.getJitter(), x.getJitter(), CLOCK_RESOLUTION + 0.01);
    EXPECT_EQ(temp.isMissing(), x.isMissing());

    // comparisons checked in OutputValueTest
  }
}
