/**
 * @file 
 * @author Denise Ratasich
 * @date 22.08.2013
 *
 * @brief Header file of the linear Kalman filter.
 */

#ifndef __ESTIMATION_KALMAN_FILTER_H__
#define __ESTIMATION_KALMAN_FILTER_H__

#include <ostream>

#include "estimation/AbstractKalmanFilter.h"
#include "estimation/Input.h"
#include "estimation/Output.h"

#include <Eigen/Dense>
using namespace Eigen;

namespace estimation 
{
  /**
   * @brief Kalman filter.
   *
   * The Kalman filter is an estimator used to estimate the state of a
   * linear dynamic system perturbed by Gaussian white noise using
   * measurements that are linear functions of the system state but
   * corrupted by additive Gaussian white noise. \cite Gre08
   *
   * This filter is only suitable for LINEAR systems, i.e. the next
   * state and output functions are linear \cite Lee11a.
   *
   * Required parameters:
   * - state transition model
   * - process noise covariance
   * - observation or measurement model
   * - measurement noise covariance
   * 
   * All other optional parameters are set to 0. 
   *
   * \note The Kalman filter needs to be validated, i.e. the method
   * \ref validate must be called to release the operation. Changing a
   * parameter needs revalidation.
   */
  class KalmanFilter : public AbstractKalmanFilter
  {
  private:
    // -----------------------------------------
    // parameters (additionally to AbstractKalmanFilter)
    // -----------------------------------------
    /** @brief State transition model. Once initialized it cannot be
     * changed. Size: n x n. */
    MatrixXd A;
    /** @brief Control input model. Size: n x l. */
    MatrixXd B;

    /** @brief Observation model. Size: m x n. */
    MatrixXd H;

  public: 
    /**
     * @brief Base constructor, the required parameters must be passed
     * with the setters.
     */
    KalmanFilter ();

    /**
     * @brief Constructor setting the required parameters.
     *
     * Minimal initialization of the Kalman filter.
     *
     * @param A State transition model.
     * @param Q Process noise covariance.
     * @param H Observation model.
     * @param R Measurement noise covariance.
     */
    KalmanFilter (MatrixXd A,
		  MatrixXd Q,
		  MatrixXd H,
		  MatrixXd R);

    /**
     * @brief Destructor of this class.
     */
    ~KalmanFilter ();

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /** 
     * @brief Sets the state transition model.
     *
     * The state transition model relates the previous state to the
     * current state. It should be calculated out of the state space
     * model F of a system which is described by difference
     * equations. You should only use this function once (for
     * initialization).
     *
     * @param A The state transition model.
     */
    void setStateTransitionModel (MatrixXd& A);

    /** 
     * @brief Sets the control input model.
     *
     * The control input model relates the control input to the
     * current state. Its default value is the zero matrix, i.e. no
     * control input accepted.
     *
     * @param B The control input model.
     */
    void setControlInputModel (MatrixXd& B);

    /** 
     * @brief Sets the observation model.
     *
     * The observation model relates the measurements to the states.
     *
     * @param H Observation model.
     */
    void setObservationModel (MatrixXd& H);

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
