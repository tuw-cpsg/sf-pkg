/**
 * @file 
 * @author Denise Ratasich
 * @date 24.07.2013
 *
 * @brief Implementation of a (simple) moving median filter.
 */

#include "estimation/MovingMedian.h"
#include <algorithm>

namespace estimation 
{

  MovingMedian::MovingMedian ()
    : out(estimation::OutputValue())
  {
    // set parameters to default values
    windowSize = 3;
  }

  MovingMedian::~MovingMedian () 
  {
    // no pointers as members, hence nothing to do
  }
	
  // -----------------------------------------
  // setting parameters
  // -----------------------------------------
  void MovingMedian::setWindowSize (unsigned int windowSize)
  {
    if (windowSize > 0)
      this->windowSize = windowSize;
  }

  // -----------------------------------------
  // IEstimationMethod implementation
  // -----------------------------------------
  Output MovingMedian::estimate (Input next)
  {
    // push the data into the queue
    in.push_back(next);
    
    // check if queue has enough elements to estimate the median with
    // the specified windowSize
    if (in.size() >= windowSize) {
      // copy data into an array for sorting
      InputValue* values = new InputValue[windowSize];
      for (int i = 0; i < windowSize; i++)
	values[i] = in[i].getInputValue();

      // sort array
      std::sort(values, values+windowSize);

      // get median of array and create output value, the estimate
      OutputValue estimate(values[windowSize/2].getValue(),
			   0,
			   values[windowSize/2].getJitter());
      out.setOutputValue(estimate);

      // delete oldest element for the next estimation
      in.pop_front();
      delete[] values;
    }
    
    return out;
  }

  Output MovingMedian::getLastEstimate(void) 
  {
    return out;
  }
}
