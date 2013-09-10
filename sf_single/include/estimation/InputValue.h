/**
 * @file 
 * @author Denise Ratasich
 * @date 24.07.2013
 *
 * @brief Header file of an input value of an estimation algorithm.
 */

#ifndef ESTIMATION_INPUTVALUE_H
#define ESTIMATION_INPUTVALUE_H

#include <ctime>

namespace estimation 
{

  /**
   * @brief Holds a data value to use for estimation of an entity.
   */
  class InputValue 
  { 
    /** @brief Stores a data value of an entity. */
    double value;
    /** @brief The jitter in ms from sensing till creating this
     * instance. */
    unsigned int jitter_ms;

    /** @brief Timestamp of creation (for updating the jitter when
     * using this value). */
    clock_t t_creation;

  public: 
    /**
     * @brief Basic constructor.
     *
     * This constructor initializes the members with default
     * values. The default values are -1 for the value, 0 for the
     * jitter.
     */
    InputValue(void);

    /**
     * @brief Constructor of this class which can be used when the
     * jitter isn't known (the jitter will be set to 0).
     *
     * @param value A data value of an entity.
     */
    InputValue(double value);

    /**
     * @brief Constructor of this class which should be used to create
     * a new value to pass to an estimation algorithm.
     *
     * @param value A data value of an entity.
     * @param jitter_ms The jitter from measuring the value till calling
     * this constructor.
     */
    InputValue(double value, unsigned int jitter_ms);

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
     * @brief Returns the jitter in ms between sensing and this
     * function call.
     */
    unsigned int getJitter(void) const;

    /**
     * @brief Sets the value, resets this InputValue (jitter_ms is set
     * to 0).
     *
     * \note This function can be used instead of creating a new
     * instance of this class. \c setJitter() can be called to update
     * the jitter associated with the new value.
     *
     * @param value The data value.
     */
    void setValue(double value);

    /**
     * @brief Sets the jitter in ms between sensing and this function
     * call.
     *
     * \note This function must be called after \c setValue()! The
     * function \c setValue() resets an instance of \c InputValue,
     * i.e. jitter_ms would be set to 0.
     *
     * @param jitter_ms The jitter in ms.
     */
    void setJitter(unsigned int jitter_ms);

    // -----------------------------------------
    // overloading operators
    // -----------------------------------------
    /**
     * @brief Swaps the data of two elements.
     */
    void swap(InputValue& first, InputValue& second);

    /**
     * @brief Overloads the assign operator.
     */
    InputValue& operator=(InputValue right);

    /**
     * @brief Overloads the == (equal) operator (compares the value
     * member).
     */
    bool operator==(const InputValue& rhs) const;

    /**
     * @brief Overloads the != (not-equal) operator (compares the
     * value member).
     */
    bool operator!=(const InputValue& rhs) const;

    /**
     * @brief Overloads the < (less-than) operator (compares the value
     * member).
     */
    bool operator<(const InputValue& rhs) const;

    /**
     * @brief Overloads the > (greater-than) operator (compares the
     * value member).
     */
    bool operator>(const InputValue& rhs) const;

    /**
     * @brief Overloads the <= (less-or-equal) operator (compares the
     * value member).
     */
    bool operator<=(const InputValue& rhs) const;

    /**
     * @brief Overloads the >= (greater-or-equal) operator (compares
     * the value member).
     */
    bool operator>=(const InputValue& rhs) const;
  };

}

#endif
