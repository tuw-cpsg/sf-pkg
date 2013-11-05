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
   * @brief Holds a data value to use for estimation of an entity.
   */
  class OutputValue 
  { 
    /** @brief Stores a data value of an entity. */
    double value;
    /** @brief Stores the variance of the data value. */
    double variance;
    /** @brief The jitter in ms from sensing till creating this
     * instance. */
    unsigned int jitter_ms;

    /** @brief Timestamp of creation (for updating the jitter when
     * using this value). */
    clock_t t_creation;

  public: 
    /**
     * @brief Basic constructor which initializes the member with defaults.
     *
     * \c value and \c variance are initialized with -1. The \c
     * jitter_ms is set to 0.
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

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /**
     * @brief Returns the value.
     *
     * @return The data value.
     */
    double getValue(void) const;

    /**
     * @brief Returns the variance (maximum deviation around the value).
     *
     * @return The variance.
     */
    double getVariance(void) const;

    /**
     * @brief Returns the jitter in ms between measuring and this function
     * call.
     */
    unsigned int getJitter(void) const;

    /**
     * @brief Sets the value, resets this OutputValue (jitter_ms is
     * set to 0).
     *
     * \note This function can be used instead of creating a new
     * instance of this class. \c setJitter() can be called to update
     * the jitter associated with the new value.
     *
     * @param value The data value.
     */
    void setValue(double value);

    /**
     * @brief Sets the variance (square of standard deviation around
     * the value).
     *
     * \note This function must be called after \c setValue()! The
     * function \c setValue() resets an instance of \c OutputValue,
     * i.e. variance would be set to -1.
     *
     * @param variance The variance.
     */
    void setVariance(double variance);

    /**
     * @brief Sets the jitter in ms between calculating the value and
     * creation of this OutputValue.
     *
     * \note This function must be called after \c setValue()! The
     * function \c setValue() resets an instance of \c OutputValue,
     * i.e. jitter_ms would be set to 0.
     *
     * @param The jitter in ms.
     */
    void setJitter(unsigned int jitter_ms);

    // -----------------------------------------
    // overloading operators
    // -----------------------------------------
    /**
     * @brief Swaps the data of two elements.
     */
    void swap(OutputValue& first, OutputValue& second);

    /**
     * @brief Overloads the assign operator.
     */
    OutputValue& operator=(OutputValue right);

    /**
     * @brief Overloads the == (equal) operator (compares the value
     * member, the variance is not taken into account!).
     */
    bool operator==(const OutputValue& rhs) const;

    /**
     * @brief Overloads the != (not-equal) operator (compares the
     * value member, the variance is not taken into account!).
     */
    bool operator!=(const OutputValue& rhs) const;

    /**
     * @brief Overloads the < (less-than) operator (compares the value
     * member, the variance is not taken into account!).
     */
    bool operator<(const OutputValue& rhs) const;

    /**
     * @brief Overloads the > (greater-than) operator (compares the
     * value member, the variance is not taken into account!).
     */
    bool operator>(const OutputValue& rhs) const;

    /**
     * @brief Overloads the <= (less-or-equal) operator (compares the
     * value member, the variance is not taken into account!).
     */
    bool operator<=(const OutputValue& rhs) const;

    /**
     * @brief Overloads the >= (greater-or-equal) operator (compares
     * the value member, the variance is not taken into account!).
     */
    bool operator>=(const OutputValue& rhs) const;
  };

}

#endif
