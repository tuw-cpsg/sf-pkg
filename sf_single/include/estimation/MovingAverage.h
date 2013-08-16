/**
 * @file 
 * @author Denise Ratasich
 * @date 16.08.2013
 *
 * @brief Header file of a (weighted) moving average filter.
 */

#ifndef __ESTIMATION_MOVING_AVERAGE_H__
#define __ESTIMATION_MOVING_AVERAGE_H__

#include "estimation/IEstimationMethod.h"
#include "estimation/InputValue.h"
#include "estimation/OutputValue.h"
#include <deque>

namespace estimation 
{
  /**
   * @brief Moving Average Filter (low-pass filter).
   *
   * Averaging = low-pass filter (-> smoothing); use when fluctuations
   * are normally distributed; 
   * 
   * Averaging in general: 
   *
   * \f[ y[n] = \sum_{k=1}^N a_k \, y[n-k] + \sum_{k=0}^N b_k \,
   * x[n-k] \f]
   * 
   * - \f$x[n-k]\f$ ... samples (input values), #in
   * - \f$y[n-k]\f$ ... output values, #out
   * - \f$N\f$ ... size of the "moving" window, #windowSize.
   *
   * Kind of filters you can establish with this class:
   * - FIR filters: \f$\forall k : a_k = 0 \f$ 
   * - IIR filters: \f$\exists k : a_k \ne 0\f$ (also called
   * autoregressive moving average filter);
   */
  class MovingAverage : public IEstimationMethod
  { 
    /** Array of input data. Corresponds to x[n-windowSize]..x[n]. */
    std::deque<InputValue> in;

    /** Array of output data. Corresponds to y[n-windowSize]..y[n]. */
    std::deque<OutputValue> out;

    /** True if there is an a-coefficient != 0, i.e. filter is in
     * "autoregressive mode". */
    bool isIIR;

    // -----------------------------------------
    // parameters
    // -----------------------------------------
    /** The number of data values to use for estimation. */
    unsigned int windowSize;

    /** Array of weighting coefficients b_k. Size of array =
     * windowSize. Used to weight the input values to estimate the
     * output. Default: hamming window values. **/
    double *b;

    /** Array of weighting coefficients a_k. Size of array =
     * windowSize-1. Used in IIR mode, i.e. also output values are
     * weighted. Default: all zero -> FIR. **/
    double *a;

  public: 
    /**
     * Constructor of this class.
     */
    MovingAverage ();

    /**
     * Destructor of this class.
     */
    ~MovingAverage ();

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /** 
     * Returns the window size.
     *
     * @return Window size.
     */
    unsigned int getWindowSize (void);

    /** 
     * Sets the window size.
     *
     * @param windowSize Number of input values used for estimation,
     * i.e. length of moving window over the data stream (input
     * values).
     */
    void setWindowSize (unsigned int windowSize);

    /** 
     * Sets the weighting coefficients for the input (b_k).
     *
     * @param b Array of weighting coefficients for input. Size of
     * array #size must equal the window size.
     */
    void setWeightingCoefficientsIn (double *b, unsigned int size);

    /** 
     * Sets the weighting coefficients for the output (a_k).
     *
     * @param a Array of weighting coefficients for output. Size of
     * array #size must equal the window size.
     */
    void setWeightingCoefficientsOut (double *a, unsigned int size);

    // -----------------------------------------
    // IEstimationMethod implementation
    // -----------------------------------------
    /**
     * Returns an estimate calculated with the given new data value.
     */
    OutputValue estimate (InputValue next);

    /**
     * Returns the last estimated value.
     */
    OutputValue getLastEstimate (void);
  };

}

#endif
