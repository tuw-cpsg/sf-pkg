/**
 * @file 
 * @author Denise Ratasich
 * @date 17.09.2013
 *
 * @brief Header file of the unscented Kalman filter.
 */

#ifndef __ESTIMATION_UNSCENTED_KALMAN_FILTER_H__
#define __ESTIMATION_UNSCENTED_KALMAN_FILTER_H__

#include <ostream>

#include "estimation/AbstractKalmanFilter.h"
#include "estimation/Input.h"
#include "estimation/Output.h"
#include "estimation/ITransformer.h"
#include "estimation/models.h"

#include <Eigen/Dense>
using namespace Eigen;

namespace estimation 
{
  /**
   * @brief Unscented Kalman Filter.
   *
   * The unscented Kalman filter is a Kalman filter which can handle
   * nonlinear systems. In contrast to EKF the models are not
   * linearized, so the original formulas of the system are
   * used. Further they differentiate in the way the parameters of
   * distribution of the state are propagated. UKF uses a sampling
   * approach called Unscented Transform. For more details, see \ref
   * unscentedkalmanfilter.
   *
   * Required parameters:
   * - state transition model f
   * - process noise covariance
   * - observation or measurement model h
   * - measurement noise covariance
   * 
   * All other optional parameters are set to 0. 
   *
   * \note The unscented Kalman filter needs to be validated, i.e. the
   * method \ref validate must be called to release the operation.
   */
  class UnscentedKalmanFilter : public AbstractKalmanFilter, public ITransformer
  {
  private:
    enum ut_functionType {
      STATE_TRANSITION_MODEL,
      OBSERVATION_MODEL
    };

    /** @brief Determines the model which should be used by the
     * Unscented Transform. */
    enum ut_functionType ut_function;

    // -----------------------------------------
    // parameters
    // -----------------------------------------
    /** 
     * @brief Probably nonlinear function which represents the state
     * transition model, calculates the a priori state vector.
     *
     * @param x The state x at time k-1 (input) used to calculate the
     * estimated (a priori) state vector at time k (output).
     * @param[in] The control input at time k-1, i.e. the current
     * control input.
     */
    func_f f;

    /** 
     * @brief Probably nonlinear function which represents the
     * observation model, calculates the estimated measurement vector.
     *
     * @param[out] z The estimated measurement vector.
     * @param[in] x The a priori state vector.
     */
    func_h h;

  public: 
    /**
     * @brief Base constructor, the required parameters must be passed
     * with the setters.
     */
    UnscentedKalmanFilter ();

    /**
     * @brief Destructor of this class.
     */
    ~UnscentedKalmanFilter ();

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /** 
     * @brief Sets the state transition model, a callback function
     * which calculates the next state.
     *
     * @param f The state transition model.
     */
    void setStateTransitionModel (func_f f);

    /** 
     * @brief Sets the observation model, a callback function which
     * calculates the estimated measurement vector according to the
     * current state.
     *
     * @param h Observation model.
     */
    void setObservationModel (func_h h);

    /**
     * @brief Releases the Kalman filter if the parameters are
     * correct. 
     *
     * Throws on error. If the validation completes successfully the
     * method \ref estimate can be used.
     */
    void validate (void);

    // -----------------------------------------
    // IEstimator implementation
    // -----------------------------------------
    /**
     * @brief Returns an estimate calculated with the given new data
     * value.
     *
     * When calling without prior validation (method \ref validate) this
     * function returns a default Output object (a single entity with
     * its value=-1).
     */
    Output estimate (Input next);

    /**
     * @brief Prints (debug) information of this Kalman filter
     * (current states of vectors and matrices).
     */
    void serialize(std::ostream& os) const;

    // -----------------------------------------
    // ITransformer implementation
    // -----------------------------------------
    /**
     * @brief Propagates a sigma point through the state transition
     * model or measurement model respectively.
     *
     * Called by an UnscentedTransform object when this UKF is passed
     * as the transformer. 
     *
     * Because this UKF needs more than one UT, with different
     * transformations, the function can be selected with the member
     * variable ut_function. Select it before calling compute() of a
     * UnscentedTransform object.
     *
     * Propagating a state (the sigmaPoint here) through the state
     * transition model, means doing the time update for the UKF.
     * Propagating a state (the sigmaPoint here) through the
     * measurement model, means calculating the expected measurement.
     *
     * @param sigmaPoint The sigma point representing a possible state
     * which should be propagated through a model.
     * @return The expected state or measurement according
     * ut_function.
     */
    VectorXd transform (const VectorXd& x);
  };

}

#endif
