/**
 * @file 
 * @author Denise Ratasich
 * @date 27.08.2013
 *
 * @brief Header file of the input of an estimation algorithm.
 */

#ifndef __ESTIMATION_INPUT_H__
#define __ESTIMATION_INPUT_H__

#include "estimation/EstimatorInterface.h"
#include "estimation/InputValue.h"

namespace estimation 
{

  /**
   * @brief State of the input interface of an estimation algorithm,
   * i.e. collects the input entities or measured entities
   * respectively.
   *
   * Holds the data values to use for estimation of an entity. The
   * input represents one or more measurements which are saved in a
   * vector of \c InputValue. When only one \c InputValue is needed,
   * e.g. for MovingAverage or MovingMedian, the size of the vector is
   * 1.
   *
   * This class provides only specific constructors, the whole work is
   * done in \c EstimationMethodInterface.
   */
  class Input : public EstimatorInterface<InputValue>
  { 
  public: 
    /*
     * @brief Basic constructor.
     *
     * Empty. No initializations. No entities to estimate (\c
     * InputValue).
     */
    Input(void);

    /**
     * @brief Constructor of this class which adds a single \c
     * InputValue to this input.
     *
     * Use it when your estimation algorithm works only with single
     * input values (e.g. one measurement per iteration), e.g. \c
     * MovingAverage or \c MovingMedian.
     *
     * @param inputValue Input entity.
     */
    Input(InputValue inputValue);

    /**
     * @brief Constructor of this class which adds several input
     * values for an estimation algorithm.
     *
     * Use it when your estimation algorithm is able to accept more
     * than one data value, e.g. \c KalmanFilter.
     *
     * @param inputValues Several input entities.
     */
    Input(std::vector<InputValue> inputValues);
  };

}

#endif
