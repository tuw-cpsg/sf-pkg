/**
 * @file 
 * @author Denise Ratasich
 * @date 22.08.2013
 *
 * @brief Header file of the linear Kalman filter.
 */

#ifndef __ESTIMATION_KALMAN_FILTER_H__
#define __ESTIMATION_KALMAN_FILTER_H__

#include "estimation/IEstimationMethod.h"
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
   * \ref validate must be called to release the operation.
   */
  class KalmanFilter : public IEstimationMethod
  {
    /** @brief Kalman gain. Size: n x m. */
    MatrixXd K;

    /** @brief Flag indicating if the parameters were checked and are
     * OK.  
     *
     * \sa function \c validate 
     * \sa for more on parameters see \ref kalmanfilter
     */
    bool validated;

    /** @brief Output of this estimation method (state and
     * variance). Size: n. */
    Output out;

    // -----------------------------------------
    // parameters
    // -----------------------------------------
    /** @brief State transition model. Once initialized it cannot be
     * changed. Size: n x n. */
    MatrixXd A;
    /** @brief Control input. Size: r. */
    VectorXd u;
    /** @brief Control input model. Size: n x r. */
    MatrixXd B;
    /** @brief Process noise covariance. Size: r x r. */
    MatrixXd Q;

    /** @brief Observation model. Size: m x n. */
    MatrixXd H;
    /** @brief Measurement noise covariance. Size: m x m. */
    MatrixXd R;

    // initial values x and P can be set
    /** @brief Estimated state vector. Size: n. */
    VectorXd x;
    /** @brief Error covariance. Size: n x n. */
    MatrixXd P;

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
    KalmanFilter (std::vector< std::vector<double> > A,
		  std::vector< std::vector<double> > Q,
		  std::vector< std::vector<double> > H,
		  std::vector< std::vector<double> > R);

    /**
     * @brief Destructor of this class.
     */
    ~KalmanFilter ();

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /**
     * @brief Returns the current state (estimated).
     *
     * @return The current state (estimated).
     */
    std::vector<double> getState () const;

    /** 
     * @brief Sets the initial state.
     *
     * @param x0 Initial state (a state is represented by the relevant
     * variables, hence a state is a vector in general).
     */
    void setInitialState (std::vector<double>& x0);

    /** 
     * @brief Sets the initial error covariance.
     *
     * Optional, the error covariance will stabilize quickly.  
     *
     * @param P0 Initial error covariance.
     */
    void setInitialErrorCovariance(std::vector< std::vector<double> >& P0);

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
    void setStateTransitionModel (std::vector< std::vector<double> >& A);

    /** 
     * @brief Sets the control input model.
     *
     * The control input model relates the control input to the
     * current state. Its default value is the zero matrix, i.e. no
     * control input accepted.
     *
     * @param B The control input model.
     */
    void setControlInputModel (std::vector< std::vector<double> >& B);
    
    /** 
     * @brief Sets the control input.
     *
     * The control input is an optional vector.
     *
     * @param u The control input.
     */
    void setControlInput (std::vector<double>& u);

    /** 
     * @brief Sets the process noise covariance.
     *
     * Determination of process noise covariance is not that simple,
     * so its best to put here "enough" uncertainty, i.e. rely more on
     * the measurements.
     *
     * @param Q Process noise covariance.
     */
    void setProcessNoiseCovariance (std::vector< std::vector<double> >& Q);

    /** 
     * @brief Sets the observation model.
     *
     * The observation model relates the measurements to the states.
     *
     * @param H Observation model.
     */
    void setObservationModel (std::vector< std::vector<double> >& H);

    /** 
     * @brief Sets the measurement noise covariance.
     *
     * Can be determined by taking "offline" samples of the
     * measurements to get the variance e.g. of a sensor.
     *
     * @param R Measurement noise covariance.
     */
    void setMeasurementNoiseCovariance (std::vector< std::vector<double> >& R);

    /**
     * @brief Releases the Kalman filter if the parameters are
     * correct. 
     *
     * Throws on error. If the validation completes successfully the
     * method \ref estimate can be used.
     */
    void validate (void);

    // -----------------------------------------
    // IEstimationMethod implementation
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
     * @brief Returns the last estimated value.
     */
    Output getLastEstimate (void);

  private:
    /**
     * @brief Copies a standard vector into the vector representation
     * of Eigen.
     *
     * Used at initialization. This class is initialized with standard
     * containers (vector).
     *
     * @param src The source vector.
     * @param dest The destination vector.
     */
    void copy(std::vector<double>& src, VectorXd& dest);

    /**
     * @brief Copies a matrix represented by a vector of vectors to
     * the matrix representation of Eigen.
     *
     * Used at initialization. This class is initialized with standard
     * containers (vector).
     *
     * @param src The source matrix.
     * @param dest The destination matrix.
     */
    void copy(std::vector< std::vector<double> >& src, MatrixXd& dest);

    /**
     * @brief Puts the estimated state and its variance into an \c
     * Output object to match the interface \c IEstimationMethod.
     *
     * TODO: fill jitter_ms
     */
    void updateOutput(void);
  };

}

#endif
