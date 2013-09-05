/**
 * @file 
 * @author Denise Ratasich
 * @date 27.08.2013
 *
 * @brief Header file of the output of an estimation algorithm.
 */

#ifndef __ESTIMATION_OUTPUT_H__
#define __ESTIMATION_OUTPUT_H__

#include "estimation/EstimatorInterface.h"
#include "estimation/OutputValue.h"

namespace estimation 
{

  /**
   * @brief State of the output interface of an estimation algorithm,
   * i.e. collects the output entities or calculated estimates
   * respectively.
   *
   * Holds the result of an estimation method, i.e. the estimated
   * state vector. The output represents one or more estimated
   * entities which are saved in a vector of \c OutputValue. When only
   * one \c OutputValue is needed, e.g. for MovingAverage or
   * MovingMedian, the size of the vector is 1.
   *
   * This class provides only specific constructors, the whole work is
   * done in \c EstimatorInterface.
   */
  class Output : public EstimatorInterface<OutputValue>
  { 
  public:
    /**
     * @brief Basic Constructor.
     *
     * Empty. No initializations. No estimated entities (\c
     * OutputValue).
     */
    Output(void);

    /**
     * @brief Constructor of this class which adds a single \c
     * OutputValue to this output.
     *
     * Use it when your estimation algorithm works only with single
     * values, i.e. one entity to estimate, e.g. \c MovingAverage or
     * \c MovingMedian.
     *
     * @param outputValue Estimated entity.
     */
    Output(OutputValue outputValue);

    /**
     * @brief Constructor of this class which adds several output
     * values of an estimation algorithm.
     *
     * Use it when your estimation algorithm is able to estimate more
     * than one entity, e.g. \c KalmanFilter.
     *
     * @param outputValues Several estimated entities.
     */
    Output(std::vector<OutputValue> outputValues);
  };

}

#endif
