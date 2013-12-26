/**
 * @file 
 * @author Denise Ratasich
 * @date 24.07.2013
 *
 * @brief Header file of the output value of an estimation algorithm.
 */

#ifndef __ESTIMATION_OUTPUTVALUE_H__
#define __ESTIMATION_OUTPUTVALUE_H__

#include "estimation/Value.h"
#include <ctime>

namespace estimation 
{
  /**
   * @brief An output data value for an estimator.
   */
  class OutputValue : public Value
  { 
    // no additional members

  public:
    /**
     * @brief Basic constructor which initializes the member with defaults.
     *
     * \c value is initialized with -1. The \c variance and \c
     * jitter_ms are set to 0.
     */
    OutputValue(void);

    /**
     * @brief Constructor of this class which represents the result of
     * an estimation algorithm.
     *
     * @param value A data value of an entity.
     * @param variance The possible deviation around the value.
     * @param jitter_ms The jitter from measuring the value till calling
     * this constructor.
     */
    OutputValue(double value, double variance, unsigned int jitter_ms);
  };

}

#endif
