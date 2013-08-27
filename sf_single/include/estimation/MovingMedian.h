/**
 * @file 
 * @author Denise Ratasich
 * @date 24.07.2013
 *
 * @brief Header file of a (simple) moving median filter.
 */

#ifndef __ESTIMATION_MOVING_MEDIAN_H__
#define __ESTIMATION_MOVING_MEDIAN_H__

#include "estimation/IEstimationMethod.h"
#include "estimation/InputValue.h"
#include "estimation/OutputValue.h"
#include <deque>

namespace estimation 
{

  class MovingMedian : public IEstimationMethod
  { 
    /** Input data. */
    std::deque<Input> in;
    /** Result of an estimation, i.e. representation of an
     * estimate. */
    OutputValue out;

    /** The number of data values to use for estimation. */
    unsigned int windowSize;

  public: 
    /**
     * Constructor of this class.
     *
     */
    MovingMedian ();

    /**
     * Destructor of this class.
     */
    ~MovingMedian ();

    /** 
     * Sets the window size.
     *
     * @param windowSize Number of input values used for estimation,
     * i.e. length of moving window over the data stream (input
     * values).
     */
    void setWindowSize (unsigned int windowSize);

    /**
     * Returns an estimate calculated with the given new data value.
     */
    OutputValue estimate (Input next);

    /**
     * Returns the last estimated value.
     */
    OutputValue getLastEstimate (void);
  };

}

#endif
