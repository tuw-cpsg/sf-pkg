/**
 * @file 
 * @author Denise Ratasich
 * @date 04.09.2013
 *
 * @brief Operator implementations for IEstimator.
 */

#include <ostream>
#include "estimation/IEstimator.h"

namespace estimation 
{
  /**
   * @brief Overloads the << operator to print a description of the
   * estimator.
   */
  std::ostream& operator<<(std::ostream& os, const IEstimator& estimator)
  {
    estimator.serialize(os);
    return os;
  }
}

