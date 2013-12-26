/**
 * @file 
 * @author Denise Ratasich
 * @date 24.07.2013
 *
 * @brief Implementation of the output value of an estimation
 * algorithm.
 */

#include "estimation/OutputValue.h"
#include <algorithm>

namespace estimation 
{
  OutputValue::OutputValue(void)
  {
    // everything done in the base class constructor
  }

  OutputValue::OutputValue(double value, double variance, unsigned int jitter_ms)
    : Value(value, variance, jitter_ms)
  {
    // everything done in the base class constructor
  }
}

