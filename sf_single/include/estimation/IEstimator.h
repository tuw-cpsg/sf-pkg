/**
 * @file 
 * @author Denise Ratasich
 * @date 25.07.2013
 *
 * @brief Interface for classes implementing an estimation method.
 */

#ifndef __ESTIMATION_IESTIMATOR_H__
#define __ESTIMATION_IESTIMATOR_H__

#include <ostream>

#include "estimation/Input.h"
#include "estimation/Output.h"

namespace estimation 
{
  /** 
   * @brief Interface for all estimation algorithms.
   */
  class IEstimator {

  public:
    /** @brief Destructor. */
    virtual ~IEstimator () { };

    /** 
     * @brief Triggers an estimation cycle with new data and returns
     * an estimate.
     *
     * @param next The new data.
     */
    virtual Output estimate (Input next) = 0;

    /**
     * @brief Prints a description fo the estimator.
     */
    virtual void serialize(std::ostream& os) const = 0;
  };

  std::ostream& operator<<(std::ostream& os, const IEstimator& estimator);
}

#endif
