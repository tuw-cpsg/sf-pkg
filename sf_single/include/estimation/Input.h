/**
 * @file 
 * @author Denise Ratasich
 * @date 27.08.2013
 *
 * @brief Header file of the input of an estimation algorithm.
 */

#ifndef ESTIMATION_INPUT_H
#define ESTIMATION_INPUT_H

#include "estimation/InputValue.h"
#include <vector>

namespace estimation 
{

  /**
   * @brief Collects the input values for an estimation algorithm.
   *
   * Holds the data values to use for estimation of an entity. The
   * input represents one or more measurements which are saved in a
   * vector of \c InputValue. When only one \c InputValue is needed,
   * e.g. for MovingAverage or MovingMedian, the size of the vector is
   * 1.
   */
  class Input 
  { 
    /** Stores the data of entities. */
    std::vector<InputValue> data;

  public: 
    /*
     * @brief Basic constructor.
     *
     * DON'T USE. This constructor does nothing. Passing this empty
     * input to an estimation method would rise an exception.
     */
    //Input(void);

    /**
     * @brief Constructor of this class which adds a single \c
     * InputValue to this input.
     *
     * Use it when your estimation algorithm works only with single
     * input values (e.g. one measurement per iteration), e.g. \c
     * MovingAverage or \c MovingMedian.
     *
     * @param inputValue Data value of an entity.
     */
    Input(InputValue inputValue);

    /**
     * @brief Constructor of this class which adds several input
     * values for an estimation algorithm.
     *
     * Use it when your estimation algorithm is able to accept more
     * than one data value, e.g. \c KalmanFilter.
     *
     * @param inputValues Input data of several entities.
     */
    Input(std::vector<InputValue> inputValues);

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /**
     * @brief Returns a \c InputValue.
     *
     * @param id The position of the \c InputValue (e.g. a
     * measurement) in the vector.
     */
    InputValue getInputValue(unsigned int id);

    /**
     * @brief Returns the first \c InputValue.
     *
     * You can use it if you have only a single \c InputValue,
     * e.g. when using an estimation algorithm where only one data
     * value is accepted and not a vector.
     */
    InputValue getInputValue();

    /**
     * @brief Returns the data values.
     *
     * Collects the values of the \c InputValue vector and returns 
     *
     * @return The data values.
     */
    std::vector<double> getValues(void);

    /**
     * @brief Returns the value of the first entity (type \c
     * InputValue) stored.
     *
     * You can use it if you have only a single \c InputValue to
     * store, e.g. when using an estimation algorithm where only one
     * data value is accepted and not a vector.
     */
    double getValue();

    /**
     * @brief Sets the \c InputValue on a specific position of this
     * input vector.
     *
     * Throws an out_of_range exception if the id is invalid. This
     * method should only be used to change an input value, not to
     * create or add one to the input vector. Use the constructor \c
     * Input(std::vector<InputValue> inputValues) to specify the size
     * of the input vector for an estimation algorithm.
     *
     * @param value The data value.
     */
    void setInputValue(InputValue value, unsigned int id);

    /**
     * @brief Sets the \c InputValue of the first element in this
     * input vector.
     *
     * You can use it if you have only a single \c InputValue to
     * store, e.g. when using an estimation algorithm where only one
     * data value is accepted and not a vector.
     */
    void setInputValue(InputValue value);

    // -----------------------------------------
    // overloading operators
    // -----------------------------------------
    /**
     * @brief Swaps the data of two elements.
     */
    void swap(Input& first, Input& second);

    /**
     * @brief Overloads the assign operator.
     */
    Input& operator=(Input right);

    /**
     * @brief Overloads the == (equal) operator (compares the values).
     */
    bool operator==(const Input& rhs) const;

    /**
     * @brief Overloads the != (not-equal) operator (compares the
     * values).
     */
    bool operator!=(const Input& rhs) const;

    // <,>,<=,>= undefined for an input vector

    /**
     * @brief Overloads the subscript operator to use this class like
     * an array.
     */
    InputValue& operator[](const int index);
  };

}

#endif
