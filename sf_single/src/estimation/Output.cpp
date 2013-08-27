/**
 * @file 
 * @author Denise Ratasich
 * @date 27.08.2013
 *
 * @brief Implementation of the output of an estimation algorithm.
 */

#include "estimation/Output.h"
#include <stdexcept>

namespace estimation 
{
  Output::Output(OutputValue outputValue)
  {
    data.push_back(outputValue);
  }

  Output::Output(std::vector<OutputValue> outputValues)
  {
    data = outputValues;
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  OutputValue Output::getOutputValue(unsigned int id)
  {
    if (id >= data.size())
      throw std::out_of_range("OutputValue with specified ID does not exist.");

    return data[id];
  }

  OutputValue Output::getOutputValue()
  {
    return data[0];
  }

  std::vector<double> Output::getValues(void)
  {
    std::vector<double> values(data.size());
    
    for (int i = 0; i < data.size(); i++)
      values[i] = data[i].getValue();

    return values;
  }

  double Output::getValue(void)
  {
    return data[0].getValue();
  }

  void Output::setOutputValue(OutputValue value, unsigned int id)
  {
    if (id >= data.size())
      throw std::out_of_range("OutputValue with specified ID does not exist.");

    data[id] = value;
  }

  void Output::setOutputValue(OutputValue value)
  {
    data[0] = value;
  }

  // -----------------------------------------
  // overloading operators
  // -----------------------------------------
  void Output::swap(Output& first, Output& second) // nothrow
  {
    // enable ADL (not necessary in our case, but good practice)
    using std::swap; 

    // by swapping the members of two classes,
    // the two classes are effectively swapped
    swap(first.data, second.data);
  }

  Output& Output::operator=(Output right)
  {
    swap(*this, right);
    return *this;
  }

  bool Output::operator==(const Output& rhs) const 
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

  bool Output::operator!=(const Output& rhs) const 
  { 
    return !(*this).operator==(rhs);
  }

  OutputValue& Output::operator[](const int index)
  {
    if (index < 0  ||  index >= data.size())
      throw std::out_of_range("Index out of range (Output.data).");

    return data[index];
  }
}

