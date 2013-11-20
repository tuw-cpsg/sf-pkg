/**
 * @file 
 * @author Denise Ratasich
 * @date 27.08.2013
 *
 * @brief Implementation of the output of an estimation algorithm.
 *
 * Basically there is nothing to implement. This class inherits from
 * \c EstimatorInterface which does all the manipulation of
 * this output interface.
 */

#include "estimation/Output.h"

namespace estimation 
{
  Output::Output(void)
  {
    // nothing to do
  }

  Output::Output(OutputValue outputValue)
    : EstimatorInterface<OutputValue>(outputValue)
  {
    // nothing to do (everything is done by the superclass)
  }

  Output::Output(std::vector<OutputValue> outputValues)
    : EstimatorInterface<OutputValue>(outputValues)
  {
    // nothing to do (everything is done by the superclass)
  }
}

