/**
 * @file 
 * @author Denise Ratasich
 * @date 16.08.2013
 *
 * @brief Header file of a (weighted) moving average filter.
 */

#ifndef __ESTIMATION_MOVING_AVERAGE_H__
#define __ESTIMATION_MOVING_AVERAGE_H__

#include <deque>
#include <ostream>

#include "estimation/IEstimator.h"
#include "estimation/Input.h"
#include "estimation/Output.h"

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
  class MovingAverage : public IEstimator
  { 
    /** @brief Array of input data. Corresponds to
     * x[n-windowSize]..x[n]. */
    std::deque<Input> in;

    /** @brief Array of output data. Corresponds to
     * y[n-windowSize]..y[n]. */
    std::deque<Output> out;

    /** @brief True if there is an a-coefficient != 0, i.e. filter is
     * in "autoregressive mode". */
    bool isIIR;

    // -----------------------------------------
    // parameters
    // -----------------------------------------
    /** @brief The number of data values to use for estimation. */
    unsigned int windowSize;

    /** @brief Array of weighting coefficients b_k. Size of array =
     * windowSize. Used to weight the input values to estimate the
     * output. Default: hamming window values. **/
    double *b;

    /** @brief Array of weighting coefficients a_k. Size of array =
     * windowSize-1. Used in IIR mode, i.e. also output values are
     * weighted. Default: all zero -> FIR. **/
    double *a;

  public: 
    /**
     * @brief Constructor of this class.
     */
    MovingAverage ();

    /**
     * @brief Destructor of this class.
     */
    ~MovingAverage ();

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /** 
     * @brief Returns the window size.
     *
     * @return Window size.
     */
    unsigned int getWindowSize (void);

    /** 
     * @brief Sets the window size.
     *
     * @param windowSize Number of input values used for estimation,
     * i.e. length of moving window over the data stream (input
     * values).
     */
    void setWindowSize (unsigned int windowSize);

    /** 
     * @brief Sets the weighting coefficients for the input (b_k).
     *
     * @param b Array of weighting coefficients for input. Size of
     * array must equal the window size.
     * @param size Size of array b.
     */
    void setWeightingCoefficientsIn (double *b, unsigned int size);

    /** 
     * @brief Sets the weighting coefficients for the output (a_k).
     *
     * @param a Array of weighting coefficients for output. Size of
     * array must equal the window size.
     * @param size Size of array a.
     */
    void setWeightingCoefficientsOut (double *a, unsigned int size);

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
    Output getLastEstimate (void) const;

    /**
     * @brief Overloads << operator to print (debug) information of
     * this filter (current state and parameters).
     */
    void serialize(std::ostream& os) const;

  private:
    /**
     * @brief Returns the normalization factor for the weighting
     * coefficients.
     *
     * @param coefficients The array of coefficients.
     * @param size The size of the array of coefficients.
     * @return Normalization factor.
     */
    double getNormalizationFactor(const double *coefficients, unsigned int size);

    /**
     * @brief Creates and initializes the weighting coefficients with
     * default values.
     */
    void createAndInitCoefficientsWithDefaults(void);
  };

}

#endif
