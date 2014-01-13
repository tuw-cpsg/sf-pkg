/**
 * @file 
 * @author Denise Ratasich
 * @date 24.07.2013
 *
 * @brief Header file of an input value of an estimation algorithm.
 */

#ifndef __ESTIMATION_INPUTVALUE_H__
#define __ESTIMATION_INPUTVALUE_H__

#include "estimation/Value.h"

namespace estimation 
{

  /**
   * @brief Holds a data value to use for estimation of an entity.
   */
  class InputValue : public Value
  { 
    /** 
     * @brief Indicates if this InputValue is initialized. 
     * 
     * An InputValue is correctly initialized if at least the value is
     * set. Before setting the value, this member variable evaluates
     * to true. 
     *
     * With this variable missing measurements can be modeled.
     */
    bool missing;

  public: 
    /**
     * @brief Basic constructor.
     *
     * This constructor initializes the members with default
     * values. The default values are -1 for the value, 0 for the
     * jitter. Member missing is set to true.
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
     * @param variance The variance of the value.
     * @param jitter_ms The jitter from measuring the value till calling
     * this constructor.
     */
    InputValue(double value, double variance, unsigned int jitter_ms);

    /**
     * @brief Basic destructor.
     */
    ~InputValue(void);

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /**
     * @brief Sets the value, resets this InputValue (jitter_ms is
     * set to 0).
     *
     * \note This function can be used instead of creating a new
     * instance of this class. \c setJitter() can be called to update
     * the jitter associated with the new value. \c missing is set to
     * false, because a value is given.
     *
     * @param value The data value.
     */
    void setValue(double value);

    /**
     * @brief Returns true if this InputValue is uninitialized.
     *
     * @return True when this InputValue has been initialized at least
     * with a value, otherwise false.
     */
    bool isMissing(void) const;

    // -----------------------------------------
    // overloading operators
    // -----------------------------------------
    /**
     * @brief Prints an appropriate description to an output stream.
     *
     * Reimplemented from Value::serialize because additional memebers
     * should be printed.
     *
     * @param os The output stream.
     */
    void serialize(std::ostream& os) const;

    /**
     * @brief Swaps the data of two elements.
     *
     * Reimplemented from Value::swap because additional members have
     * to be swapped.
     */
    void swap(InputValue& first, InputValue& second);

    /**
     * @brief Overloads the assign operator.
     *
     * Reimplemented from Value::operator= because additional members
     * have to be swapped.
     */
    InputValue& operator=(InputValue right);
  };

}

#endif
