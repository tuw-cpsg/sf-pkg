/**
 * @file 
 * @author Denise Ratasich
 * @date 27.08.2013
 *
 * @brief Implementation of the input of an estimation algorithm.
 *
 * Basically there is nothing to implement. This class inherits from
 * \c EstimatorInterface which does all the manipulation of
 * this input interface.
 */

#include "estimation/Input.h"

namespace estimation 
{
  Input::Input(void)
  {
    // nothing to do
  }

  Input::Input(InputValue inputValue)
    : EstimatorInterface<InputValue>(inputValue)
  {
    // nothing to do (everything is done by the superclass)
  }

  Input::Input(std::vector<InputValue> inputValues)
    : EstimatorInterface<InputValue>(inputValues)
  {
    // nothing to do (everything is done by the superclass)
  }
}

