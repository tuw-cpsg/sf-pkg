/**
 * @file 
 * @author Denise Ratasich
 * @date 24.07.2013
 *
 * @brief Implementation of an input value of an estimation algorithm.
 */

#include "estimation/InputValue.h"
#include <algorithm>

namespace estimation 
{
  InputValue::InputValue(void)
  {
    // set to defaults
    this->value = -1;
    this->jitter_ms = 0;
    this->t_creation = clock();
  }

  InputValue::InputValue(double value)
  {
    this->value = value;
    this->jitter_ms = 0;
    this->t_creation = clock();
  }

  InputValue::InputValue(double value, unsigned int jitter_ms)
  {
    this->value = value;
    this->jitter_ms = jitter_ms;
    this->t_creation = clock();
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  double InputValue::getValue(void) const
  {
    return value;
  }

  unsigned int InputValue::getJitter(void) const
  {
    unsigned int curLifetime_ms;
    curLifetime_ms = float(clock() - t_creation)*1000/CLOCKS_PER_SEC;

    return jitter_ms + curLifetime_ms;
  }

  void InputValue::setValue(double value)
  {
    this->value = value;
    this->jitter_ms = 0;
    this->t_creation = clock();
  }

  void InputValue::setJitter(unsigned int jitter_ms)
  {
    this->jitter_ms = jitter_ms;
  }

  // -----------------------------------------
  // overloading operators
  // -----------------------------------------
  void InputValue::swap(InputValue& first, InputValue& second) // nothrow
  {
    // enable ADL (not necessary in our case, but good practice)
    using std::swap; 

    // by swapping the members of two classes,
    // the two classes are effectively swapped
    swap(first.value, second.value); 
    swap(first.jitter_ms, second.jitter_ms);
    swap(first.t_creation, second.t_creation);
  }

  InputValue& InputValue::operator=(InputValue right)
  {
    swap(*this, right);
    return *this;
  }

  bool InputValue::operator==(const InputValue& rhs) const 
  { 
    return (*this).value == rhs.value; 
  }

  bool InputValue::operator!=(const InputValue& rhs) const 
  { 
    return !(*this).operator==(rhs);
  }

  bool InputValue::operator<(const InputValue& rhs) const 
  { 
    return (*this).value < rhs.value; 
  }

  bool InputValue::operator>(const InputValue& rhs) const 
  { 
    return rhs.operator<(*this); 
  }

  bool InputValue::operator<=(const InputValue& rhs) const 
  { 
    return !(*this).operator>(rhs); 
  }

  bool InputValue::operator>=(const InputValue& rhs) const 
  { 
    return !(*this).operator<(rhs); 
  }
}

