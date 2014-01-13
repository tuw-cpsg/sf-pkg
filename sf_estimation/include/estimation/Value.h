/**
 * @file 
 * @author Denise Ratasich
 * @date 26.12.2013
 *
 * @brief Header file of the value (e.g. input or output) of an
 * estimation algorithm.
 */

#ifndef __ESTIMATION_VALUE_H__
#define __ESTIMATION_VALUE_H__

#include <ctime>
#include <ostream>

namespace estimation 
{
  /**
   * @brief Holds a data value to use for estimation of an entity.
   */
  class Value 
  { 
  protected:
    /** @brief Stores a data value of an entity. */
    double value;
    /** @brief Stores the variance of the data value. */
    double variance;
    /** @brief The jitter in ms from sensing till creating this
     * instance. */
    unsigned int jitter_ms;

    /** @brief Timestamp of creation (for updating the jitter when
     * using this value). */
    std::clock_t t_creation;

  public: 
    /**
     * @brief Basic constructor which initializes the member with defaults.
     *
     * \c value is initialized with -1. The \c variance and \c
     * jitter_ms are set to 0.
     */
    Value(void);

    /**
     * @brief Constructor of this class which represents the a data value, e.g. input or output, of
     * an estimation algorithm.
     *
     * @param value A data value of an entity.
     * @param variance The variance of the value.
     * @param jitter_ms The jitter from measuring the value till calling
     * this constructor.
     */
    Value(double value, double variance, unsigned int jitter_ms);

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
     * @brief Sets the value, resets this Value.
     *
     * \note This function can be used instead of creating a new
     * instance of this class. \c setVariance() and \c setJitter() can
     * be called to update the members.
     *
     * @param value The data value.
     */
    virtual void setValue(double value);

    /**
     * @brief Sets the variance (square of standard deviation around
     * the value).
     *
     * \note This function must be called after \c setValue()! The
     * function \c setValue() resets an instance of \c Value,
     * i.e. variance would be set to -1.
     *
     * @param variance The variance.
     */
    void setVariance(double variance);

    /**
     * @brief Sets the jitter in ms between calculating the value and
     * creation of this Value.
     *
     * \note This function must be called after \c setValue()! The
     * function \c setValue() resets an instance of \c Value,
     * i.e. jitter_ms would be set to 0.
     *
     * @param The jitter in ms.
     */
    void setJitter(unsigned int jitter_ms);

    // -----------------------------------------
    // overloading operators
    // -----------------------------------------
    /**
     * @brief Prints an appropriate description to an output stream.
     *
     * @param os The output stream to print to.
     */
    virtual void serialize(std::ostream& os) const;

    /**
     * @brief Swaps the data of two elements.
     */
    void swap(Value& first, Value& second);

    /**
     * @brief Overloads the assign operator.
     */
    Value& operator=(Value right);

    /**
     * @brief Overloads the == (equal) operator (compares the value
     * member, the variance is not taken into account!).
     */
    bool operator==(const Value& rhs) const;

    /**
     * @brief Overloads the != (not-equal) operator (compares the
     * value member, the variance is not taken into account!).
     */
    bool operator!=(const Value& rhs) const;

    /**
     * @brief Overloads the < (less-than) operator (compares the value
     * member, the variance is not taken into account!).
     */
    bool operator<(const Value& rhs) const;

    /**
     * @brief Overloads the > (greater-than) operator (compares the
     * value member, the variance is not taken into account!).
     */
    bool operator>(const Value& rhs) const;

    /**
     * @brief Overloads the <= (less-or-equal) operator (compares the
     * value member, the variance is not taken into account!).
     */
    bool operator<=(const Value& rhs) const;

    /**
     * @brief Overloads the >= (greater-or-equal) operator (compares
     * the value member, the variance is not taken into account!).
     */
    bool operator>=(const Value& rhs) const;
  };

  /**
   * @brief Overloads the << operator to pass an appropriate
   * description of the value to an output stream.
   */
  std::ostream& operator<<(std::ostream& lhs, const Value& rhs);
}

#endif
