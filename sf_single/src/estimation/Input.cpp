/**
 * @file 
 * @author Denise Ratasich
 * @date 27.08.2013
 *
 * @brief Implementation of the input of an estimation algorithm.
 */

#include "estimation/Input.h"
#include <stdexcept>

namespace estimation 
{
  /*
  Input::Input(void)
  {
    // nothing to do
  }
  */

  Input::Input(InputValue inputValue)
  {
    data.push_back(inputValue);
  }

  Input::Input(std::vector<InputValue> inputValues)
  {
    data = inputValues;
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  InputValue Input::getInputValue(unsigned int id)
  {
    if (id >= data.size())
      throw std::out_of_range("InputValue with specified ID does not exist.");

    return data[id];
  }

  InputValue Input::getInputValue()
  {
    return data[0];
  }

  std::vector<double> Input::getValues(void)
  {
    std::vector<double> values(data.size());
    
    for (int i = 0; i < data.size(); i++)
      values[i] = data[i].getValue();

    return values;
  }

  double Input::getValue(void)
  {
    return data[0].getValue();
  }

  void Input::setInputValue(InputValue value, unsigned int id)
  {
    if (id >= data.size())
      throw std::out_of_range("InputValue with specified ID does not exist.");

    data[id] = value;
  }

  void Input::setInputValue(InputValue value)
  {
    data[0] = value;
  }

  // -----------------------------------------
  // overloading operators
  // -----------------------------------------
  void Input::swap(Input& first, Input& second) // nothrow
  {
    // enable ADL (not necessary in our case, but good practice)
    using std::swap; 

    // by swapping the members of two classes,
    // the two classes are effectively swapped
    swap(first.data, second.data);
  }

  Input& Input::operator=(Input right)
  {
    swap(*this, right);
    return *this;
  }

  bool Input::operator==(const Input& rhs) const 
  { 
    bool equal = true;

    for (int i = 0; i < (*this).data.size(); i++) {
      if ((*this).data[i] != rhs.data[i]) {
	equal = false;
	break;
      }
    }

    return equal;
  }

  bool Input::operator!=(const Input& rhs) const 
  { 
    return !(*this).operator==(rhs);
  }

  InputValue& Input::operator[](const int index)
  {
    if (index < 0  ||  index >= data.size())
      throw std::out_of_range("Index out of range (Input.data).");

    return data[index];
  }
}

