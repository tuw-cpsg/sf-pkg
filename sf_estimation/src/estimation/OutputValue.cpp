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
    // init with defaults
    this->value = -1;
    this->variance = -1;
    this->jitter_ms = 0;
    this->t_creation = clock();
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
  double OutputValue::getValue(void) const
  {
    return value;
  }

  double OutputValue::getVariance(void) const
  {
    return variance;
  }

  unsigned int OutputValue::getJitter(void) const
  {
    unsigned int curLifetime_ms;
    curLifetime_ms = float(clock() - t_creation)*1000/CLOCKS_PER_SEC;

    return jitter_ms + curLifetime_ms;
  }

  void OutputValue::setValue(double value)
  {
    this->value = value;
    // defaults - setting the value is like creating a new instance
    this->variance = -1;
    this->jitter_ms = 0;
    this->t_creation = clock();
  }

  void OutputValue::setVariance(double variance)
  {
    this->variance = variance;
  }

  void OutputValue::setJitter(unsigned int jitter_ms)
  {
    this->jitter_ms = jitter_ms;
  }

  // -----------------------------------------
  // overloading operators
  // -----------------------------------------
  void OutputValue::swap(OutputValue& first, OutputValue& second) // nothrow
  {
    // enable ADL (not necessary in our case, but good practice)
    using std::swap; 

    // by swapping the members of two classes,
    // the two classes are effectively swapped
    swap(first.value, second.value); 
    swap(first.variance, second.variance); 
    swap(first.jitter_ms, second.jitter_ms);
    swap(first.t_creation, second.t_creation);
  }

  OutputValue& OutputValue::operator=(OutputValue right)
  {
    swap(*this, right);
    return *this;
  }

  bool OutputValue::operator==(const OutputValue& rhs) const 
  { 
    return (*this).value == rhs.value; 
  }

  bool OutputValue::operator!=(const OutputValue& rhs) const 
  { 
    return !(*this).operator==(rhs);
  }

  bool OutputValue::operator<(const OutputValue& rhs) const 
  { 
    return (*this).value < rhs.value; 
  }

  bool OutputValue::operator>(const OutputValue& rhs) const 
  { 
    return rhs.operator<(*this); 
  }

  bool OutputValue::operator<=(const OutputValue& rhs) const 
  { 
    return !(*this).operator>(rhs); 
  }

  bool OutputValue::operator>=(const OutputValue& rhs) const 
  { 
    return !(*this).operator<(rhs); 
  }
}

