/**
 * @file 
 * @author Denise Ratasich
 * @date 24.07.2013
 *
 * @brief Header file of the output value of an estimation algorithm.
 */

#ifndef ESTIMATION_OUTPUTVALUE_H
#define ESTIMATION_OUTPUTVALUE_H

#include <ctime>

namespace estimation 
{

  /**
   * Holds a data value to use for estimation of an entity.
   */
  class OutputValue 
  { 
    /** Stores a data value of an entity. */
    double value;
    /** Stores the variance of the data value. */
    double variance;
    /** The jitter in ms from sensing till creating this instance. */
    unsigned int jitter_ms;

    /** Timestamp of creation (for updating the jitter when using this
     * value). */
    clock_t t_creation;

  public: 
    /**
     * Basic constructor.
     *
     * This constructor is only to create arrays of OutputValue without
     * initializing.
     */
    OutputValue(void);

    /**
     * Constructor of this class which represents the result of an
     * estimation algorithm.
     *
     * @param value A data value of an entity.
     * @param variance The possible deviation around the value.
     * @param jitter_ms The jitter from measuring the value till calling
     * this constructor.
     */
    OutputValue(double value, double variance, unsigned int jitter_ms);

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /**
     * Returns the value.
     *
     * @return The data value.
     */
    double getValue(void);

    /**
     * Returns the variance (maximum deviation around the value).
     *
     * @return The variance.
     */
    double getVariance(void);

    /**
     * Returns the jitter in ms between measuring and this function
     * call.
     */
    unsigned int getJitter(void);

    /**
     * Sets the value.
     *
     * @param value The data value.
     */
    void setValue(double value);

    /**
     * Sets the variance (maximum deviation around the value).
     *
     * @param variance The variance.
     */
    void setVariance(double variance);

    /**
     * Sets the jitter in ms between sensing and creation of this
     * output value.
     *
     * When calling this function the time of creation is updated to
     * now! Hence calling this function is like creating a new
     * OutputValue object.
     *
     * @param The jitter in ms.
     */
    void setJitter(unsigned int jitter_ms);
  };

}

#endif
