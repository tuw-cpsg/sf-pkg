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
    this->missing = true;
  }

  InputValue::InputValue(double value)
  {
    this->setValue(value);
  }

  InputValue::InputValue(double value, double variance, unsigned int jitter_ms)
    : Value(value, variance, jitter_ms)
  {
    this->missing = false;
  }

  InputValue::~InputValue(void)
  {
    // nothing to do
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  bool InputValue::isMissing(void) const
  {
    return missing;
  }

  void InputValue::setValue(double value)
  {
    Value::setValue(value);
    missing = false;
  }

  // -----------------------------------------
  // overloading operators
  // -----------------------------------------
  void InputValue::serialize(std::ostream& os) const
  {
    if (missing)
      os << "(missing)";
    else
      Value::serialize(os);
  }

  void InputValue::swap(InputValue& first, InputValue& second) // nothrow
  {
    // enable ADL (not necessary in our case, but good practice)
    using std::swap; 

    // by swapping the members of two classes,
    // the two classes are effectively swapped
    Value::swap(first,second);			// swap members in Value
    swap(first.missing, second.missing);	// swap additional members
  }

  InputValue& InputValue::operator=(InputValue right)
  {
    swap(*this, right);
    return *this;
  }
}

