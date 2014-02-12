/**
 * @file 
 * @author Denise Ratasich
 * @date 25.07.2013
 *
 * @brief Interface for classes implementing an estimation method.
 */

#ifndef __ESTIMATION_IESTIMATOR_H__
#define __ESTIMATION_IESTIMATOR_H__

#include <ostream>

#include "estimation/Input.h"
#include "estimation/Output.h"

namespace estimation 
{
  /** 
   * @brief Interface for all estimation algorithms.
   */
  class IEstimator {

  public:
    /** @brief Destructor. */
    virtual ~IEstimator () { };

    /** 
     * @brief Triggers an estimation cycle with new data and returns
     * an estimate.
     *
     * @param next The new data.
     */
    virtual Output estimate (Input next) = 0;

    /**
     * @brief Sets the control input.
     *
     * Estimators with a state transition model can additionally set a
     * control input.
     *
     * @param u The control input.
     */
    virtual void setControlInput(Input u) { };

    /**
     * @brief Returns the last estimated value.
     */
    virtual Output getLastEstimate(void) const = 0;

    /**
     * @brief Prints a description of the estimator.
     */
    virtual void serialize(std::ostream& os) const = 0;

    /**
     * @brief Exception thrown by an estimator.
     */
    class EstimatorException : public std::runtime_error
    {
    public:
    EstimatorException(const std::string& info) 
      : std::runtime_error(info) { }
    };
    typedef EstimatorException estimator_error;
  };

  std::ostream& operator<<(std::ostream& os, const IEstimator& estimator);
}

#endif
