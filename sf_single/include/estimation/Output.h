/**
 * @file 
 * @author Denise Ratasich
 * @date 27.08.2013
 *
 * @brief Header file of the output of an estimation algorithm.
 */

#ifndef ESTIMATION_OUTPUT_H
#define ESTIMATION_OUTPUT_H

#include "estimation/OutputValue.h"
#include <vector>

namespace estimation 
{

  /**
   * @brief Collects the output values of an estimation algorithm.
   *
   * Holds the result of an estimation method, i.e. the estimated
   * state vector. The output represents one or more estimated
   * entities which are saved in a vector of \c OutputValue. When only
   * one \c OutputValue is needed, e.g. for MovingAverage or
   * MovingMedian, the size of the vector is 1.
   */
  class Output
  { 
    /** @brief Stores the data of entities. */
    std::vector<OutputValue> data;

  public:
    /**
     * @brief Constructor of this class which adds a single \c
     * OutputValue to this output.
     *
     * Use it when your estimation algorithm works only with single
     * values, i.e. one entity to estimate, e.g. \c MovingAverage or
     * \c MovingMedian.
     *
     * @param outputValue Estimated data value of an entity.
     */
    Output(OutputValue outputValue);

    /**
     * @brief Constructor of this class which adds several output
     * values of an estimation algorithm.
     *
     * Use it when your estimation algorithm is able to estimate more
     * than one entity, e.g. \c KalmanFilter.
     *
     * @param outputValues Estimated data of several entities.
     */
    Output(std::vector<OutputValue> outputValues);

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /**
     * @brief Returns an \c OutputValue.
     *
     * @param id The position of the \c OutputValue, i.e. an estimate,
     * in the vector.
     */
    OutputValue getOutputValue(unsigned int id);

    /**
     * @brief Returns the first \c OutputValue.
     *
     * You can use it if you have only a single \c OutputValue,
     * e.g. when using an estimation algorithm where only one entity
     * is estimated.
     */
    OutputValue getOutputValue();

    /**
     * @brief Returns the estimated data values.
     *
     * Collects the values of the \c OutputValue vector and returns
     * it.
     *
     * @return The estimated data values.
     */
    std::vector<double> getValues(void);

    /**
     * @brief Returns the estimated value of the first entity.
     *
     * You can use it if you have only a single \c OutputValue,
     * e.g. when using an estimation algorithm where only one entity
     * is estimated.
     */
    double getValue();

    /**
     * @brief Sets the \c OutputValue on a specific position of this
     * output vector.
     *
     * Throws an out_of_range exception if the id is invalid. This
     * method should only be used to change an output value, not to
     * create or add one to the output vector. Use the constructor \c
     * Output(std::vector<OutputValue> outputValues) to specify the
     * size of the output vector for an estimation algorithm.
     *
     * @param value The data value.
     */
    void setOutputValue(OutputValue value, unsigned int id);

    /**
     * @brief Sets the \c OutputValue of the first element in this
     * output vector.
     *
     * You can use it if you have only a single \c OutputValue,
     * e.g. when using an estimation algorithm where only one entity
     * is estimated.
     */
    void setOutputValue(OutputValue value);

    // -----------------------------------------
    // overloading operators
    // -----------------------------------------
    /**
     * @brief Swaps the data of two elements.
     */
    void swap(Output& first, Output& second);

    /**
     * @brief Overloads the assign operator.
     */
    Output& operator=(Output right);

    /**
     * @brief Overloads the == (equal) operator (compares the values).
     */
    bool operator==(const Output& rhs) const;

    /**
     * @brief Overloads the != (not-equal) operator (compares the
     * values).
     */
    bool operator!=(const Output& rhs) const;

    // <,>,<=,>= undefined for an output vector

    /**
     * @brief Overloads the subscript operator to use this class like
     * an array.
     */
    OutputValue& operator[](const int index);
  };

}

#endif
