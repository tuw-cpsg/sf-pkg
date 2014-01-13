/**
 * @file 
 * @author Denise Ratasich
 * @date 26.12.2013
 *
 * @brief Implementation of the value (e.g. input or output) of an
 * estimation algorithm.
 */

#include "estimation/Value.h"
#include <algorithm>

namespace estimation 
{
  Value::Value(void)
  {
    // init with defaults
    this->value = -1;
    this->variance = 0;
    this->jitter_ms = 0;
    this->t_creation = std::clock();
  }

  Value::Value(double value, double variance, unsigned int jitter_ms)
  {
    this->value = value;
    this->variance = variance;
    this->jitter_ms = jitter_ms;
    this->t_creation = std::clock();
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  double Value::getValue(void) const
  {
    return value;
  }

  double Value::getVariance(void) const
  {
    return variance;
  }

  unsigned int Value::getJitter(void) const
  {
    unsigned int curLifetime_ms;
    curLifetime_ms = (double)(std::clock() - t_creation)*1000/CLOCKS_PER_SEC;

    return jitter_ms + curLifetime_ms;
  }

  void Value::setValue(double value)
  {
    this->value = value;
    // defaults - setting the value is like creating a new instance
    this->variance = 0;
    this->jitter_ms = 0;
    this->t_creation = std::clock();
  }

  void Value::setVariance(double variance)
  {
    this->variance = variance;
  }

  void Value::setJitter(unsigned int jitter_ms)
  {
    this->jitter_ms = jitter_ms;
  }

  // -----------------------------------------
  // overloading operators
  // -----------------------------------------
  void Value::serialize(std::ostream& os) const
  {
    os << "(" 
       << value << ","
       << variance << ","
       << getJitter() << ")";
  }

  void Value::swap(Value& first, Value& second) // nothrow
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

  Value& Value::operator=(Value right)
  {
    swap(*this, right);
    return *this;
  }

  bool Value::operator==(const Value& rhs) const 
  { 
    return (*this).value == rhs.value; 
  }

  bool Value::operator!=(const Value& rhs) const 
  { 
    return !(*this).operator==(rhs);
  }

  bool Value::operator<(const Value& rhs) const 
  { 
    return (*this).value < rhs.value; 
  }

  bool Value::operator>(const Value& rhs) const 
  { 
    return rhs.operator<(*this); 
  }

  bool Value::operator<=(const Value& rhs) const 
  { 
    return !(*this).operator>(rhs); 
  }

  bool Value::operator>=(const Value& rhs) const 
  { 
    return !(*this).operator<(rhs); 
  }

  // not a member but it corresponds to class Value here..
  std::ostream& operator<<(std::ostream& lhs, const Value& rhs)
  {
    rhs.serialize(lhs);
    return lhs;
  }
}

