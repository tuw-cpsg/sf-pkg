/**
 * @file 
 * @author Denise Ratasich
 * @date 24.07.2013
 *
 * @brief Implementation of the output value of an estimation
 * algorithm.
 */

#include "estimation/OutputValue.h"

namespace estimation 
{

  OutputValue::OutputValue(void)
  {
    // nothing to do
  }

  OutputValue::OutputValue(double value, double variance, unsigned int jitter_ms)
  {
    this->value = value;
    this->variance = variance;
    this->jitter_ms = jitter_ms;
    this->t_creation = clock();
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  double OutputValue::getValue(void)
  {
    return value;
  }

  double OutputValue::getVariance(void)
  {
    return variance;
  }

  unsigned int OutputValue::getJitter(void)
  {
    unsigned int curLifetime_ms;
    curLifetime_ms = float(clock() - t_creation)*1000/CLOCKS_PER_SEC;

    return jitter_ms + curLifetime_ms;
  }

  void OutputValue::setValue(double value)
  {
    this->value = value;
  }

  void OutputValue::setVariance(double variance)
  {
    this->variance = variance;
  }

  void OutputValue::setJitter(unsigned int jitter_ms)
  {
    // The time of creation is updated to now! It is assumed that a
    // new value has been assigned with a different timestamp and a
    // different jitter, i.e. a completely new OutputValue object is
    // defined.
    this->t_creation = clock();
    this->jitter_ms = jitter_ms;
  }
}

