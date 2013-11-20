/**
 * @file 
 * @author Denise Ratasich
 * @date 24.07.2013
 *
 * @brief Header file of a (simple) moving median filter.
 */

#ifndef __ESTIMATION_MOVING_MEDIAN_H__
#define __ESTIMATION_MOVING_MEDIAN_H__

#include <deque>
#include <ostream>

#include "estimation/IEstimator.h"
#include "estimation/Input.h"
#include "estimation/Output.h"

namespace estimation 
{
  /**
   * @brief Moving Median Filter.
   *
   * Estimation is done by sorting the last N elements and take the
   * median of it. 
   *
   * Use this filter for sample ...
   * - where very large deviations (outliers) can occur.
   * - which are Laplace distributed.
   */
  class MovingMedian : public IEstimator
  { 
    /** @brief Input data. */
    std::deque<Input> in;
    /** @brief Result of an estimation, i.e. representation of an
     * estimate. */
    Output out;

    // -----------------------------------------
    // parameters
    // -----------------------------------------
    /** @brief The number of data values to use for estimation. */
    unsigned int windowSize;

  public: 
    /**
     * @brief Constructor of this class.
     */
    MovingMedian ();

    /**
     * @brief Destructor of this class.
     */
    ~MovingMedian ();

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /** 
     * @brief Sets the window size.
     *
     * @param windowSize Number of input values used for estimation,
     * i.e. length of moving window over the data stream (input
     * values).
     */
    void setWindowSize (unsigned int windowSize);

    // -----------------------------------------
    // IEstimator implementation
    // -----------------------------------------
    /**
     * @brief Returns an estimate calculated with the given new data
     * value.
     */
    Output estimate (Input next);

    /**
     * @brief Returns the last estimated value.
     */
    Output getLastEstimate (void);

    /**
     * @brief Prints (debug) information of this filter (current state
     * and parameters).
     */
    void serialize(std::ostream& os) const;
  };

}

#endif
