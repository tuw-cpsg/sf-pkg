/**
 * @file 
 * @author Denise Ratasich
 * @date 05.09.2013
 *
 * @brief Header file of the extended Kalman filter.
 */

#ifndef __ESTIMATION_EXTENDED_KALMAN_FILTER_H__
#define __ESTIMATION_EXTENDED_KALMAN_FILTER_H__

#include <ostream>

#include "estimation/AbstractKalmanFilter.h"
#include "estimation/Input.h"
#include "estimation/Output.h"

#include <Eigen/Dense>
using namespace Eigen;

namespace estimation 
{
  /**
   * @brief Extended Kalman filter.
   *
   * The extended Kalman filter is a Kalman filter which can handle
   * nonlinear systems. The system's representation is \em locally
   * linearized, so the system should be "smooth" enought to get good
   * results. For more details, see \ref extendedkalmanfilter.
   *
   * Required parameters:
   * - state transition model f
   * - function to calculate jacobian of f
   * - process noise covariance
   * - observation or measurement model h
   * - function to calculate jacobian of h
   * - measurement noise covariance
   * 
   * All other optional parameters are set to 0. 
   *
   * \note The extended Kalman filter needs to be validated, i.e. the
   * method \ref validate must be called to release the operation.
   */
  class ExtendedKalmanFilter : public AbstractKalmanFilter
  {
  public:
    /** Pointer to a function representing the state transition
     * model. */
    typedef void (*func_f)(VectorXd& x, const VectorXd& u);
    /** Pointer to a function calculating the Jacobian of the state
     * transition model. */
    typedef void (*func_df)(MatrixXd& A, const VectorXd& x, const VectorXd& u);
    /** Pointer to a function representing the observation model. */
    typedef void (*func_h)(VectorXd& z, const VectorXd& x);
    /** Pointer to a function calculating the Jacobian of the
     * observation model. */
    typedef void (*func_dh)(MatrixXd& H, const VectorXd& x);

  private:
    // -----------------------------------------
    // parameters
    // -----------------------------------------
    /** 
     * @brief Probably nonlinear function which represents the state
     * transition model, calculates the a priori state vector.
     *
     * @param x The state x at time k-1 (input) used to calculate the
     * estimated (a priori) state vector at time k
     * (output). 
     * @param[in] The control input at time k-1.
     */
    func_f f;
    /** 
     * @brief Calculates the Jacobian of f with respect to x_{k-1},
     * derivation of the system model to x.
     */
    func_df df;
    /** @brief Jacobian of f with respect to x_{k-1}, derivation of
     * the system model to x. Size: n x n.*/
    MatrixXd A;

    /** 
     * @brief Probably nonlinear function which represents the
     * observation model, calculates the estimated measurement vector.
     *
     * @param[out] z The estimated measurement vector.
     * @param[in] x The a priori state vector.
     */
    func_h h;
    /** 
     * @brief Calculates the Jacobian of h with respect to a priori x,
     * derivation of the observation model to x. 
     *
     * @param[out] The calculated Jacobian.
     * @param[in] The a priori state vector.
     */
    func_dh dh;
    /** @brief Jacobian of h with respect to a priori x, derivation of
     * the observation model. Size: m x n. */
    MatrixXd H;

  public: 
    /**
     * @brief Base constructor, the required parameters must be passed
     * with the setters.
     */
    ExtendedKalmanFilter ();

    /**
     * @brief Destructor of this class.
     */
    ~ExtendedKalmanFilter ();

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
     * @brief Passes the function to calculate the Jacobian of the
     * state transition model.
     *
     * @param df The pointer to the function which calculates the
     * Jacobian of the state transition model.
     */
    void setJacobianOfStateTransitionModel (func_df df);

    /** 
     * @brief Sets the observation model, a callback function which
     * calculates the estimated measurement vector according to the
     * current state.
     *
     * @param h Observation model.
     */
    void setObservationModel (func_h h);

    /** 
     * @brief Sets the pointer to the function calculating the
     * Jacobian of the observation model.
     *
     * @param dh Function calculating the Jacobian of the observation
     * model.
     */
    void setJacobianOfObservationModel (func_dh dh);

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
  };

}

#endif
