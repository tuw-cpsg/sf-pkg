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
  {
    // set parameters to default values
    windowSize = 3;

    // Add a default entity to the output interface. The members of
    // this default output entity will be changed instead of always
    // creating a new \c OutputValue.
    out.add(OutputValue());
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
      // get data for sorting 
      std::vector<InputValue> values(windowSize);
      for (int i = 0; i < windowSize; i++)
	values[i] = in[i].getValue();

      // sort array
      std::sort(values.begin(), values.end());

      // get median of array and set new values
      out[0].setValue(values[windowSize/2].getValue());
      out[0].setVariance(0);
      out[0].setJitter(values[windowSize/2].getJitter());

      // delete oldest element for the next estimation
      in.pop_front();
    }
    
    return out;
  }

  Output MovingMedian::getLastEstimate(void) 
  {
    return out;
  }

  void MovingMedian::serialize(std::ostream& os) const
  {
    os << "MovingMedian" << std::endl
       << "window-size = " << this->windowSize << std::endl
       << "input (new to old) = ";
    for (int i = 0; i < this->in.size(); i++)
      os << this->in[i].getValue() << ", ";

    os << std::endl
       << "output = " << out.getValue() << std::endl;
  }
}
