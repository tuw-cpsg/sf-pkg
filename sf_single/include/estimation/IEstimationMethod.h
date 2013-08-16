/**
 * @file 
 * @author Denise Ratasich
 * @date 25.07.2013
 *
 * @brief Interface for classes implementing an estimation method.
 */

#ifndef ESTIMATION_IESTIMATIONMETHOD_H
#define ESTIMATION_IESTIMATIONMETHOD_H

#include "estimation/InputValue.h"
#include "estimation/OutputValue.h"

namespace estimation 
{

  /** 
   * Interface for all estimation algorithms.
   */
  class IEstimationMethod {

  public:
    /** Destructor. */
    virtual ~IEstimationMethod () { };

    /** 
     * Triggers an estimation cycle with the new data value and
     * returns an estimate.
     *
     * @param next The new data value.
     */
    virtual OutputValue estimate (InputValue next) = 0;
  };

}

#endif
