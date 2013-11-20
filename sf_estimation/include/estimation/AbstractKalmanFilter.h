/**
 * @file 
 * @author Denise Ratasich
 * @date 02.10.2013
 *
 * @brief Header file of a basic Kalman filter.
 */

#ifndef __ESTIMATION_ABSTRACT_KALMAN_FILTER_H__
#define __ESTIMATION_ABSTRACT_KALMAN_FILTER_H__

#include <ostream>

#include "estimation/IEstimator.h"
#include "estimation/Input.h"
#include "estimation/Output.h"

#include <Eigen/Dense>
using namespace Eigen;

namespace estimation 
{
  /**
   * @brief Abstract Kalman Filter.
   *
   * Base of each Kalman filter, which does initialization and basic
   * validation.
   */
  class AbstractKalmanFilter : public IEstimator
  {
  protected:
    /** @brief Kalman gain. Size: n x m. */
    MatrixXd K;

    /** @brief Flag indicating if the parameters were checked and are
     * OK.  
     *
     * \sa function \c validate 
     * \sa for more on parameters see \ref extendedkalmanfilter
     */
    bool validated;

    /** @brief Output of this estimation method (state and
     * variance). Size: n. */
    Output out;

    // -----------------------------------------
    // parameters
    // -----------------------------------------
    /** @brief Control input. Size: l. */
    VectorXd u;
    /** @brief Process noise covariance. Size: n x n. */
    MatrixXd Q;
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
    AbstractKalmanFilter ();
    
    /**
     * @brief Destructor of this class.
     */
    virtual ~AbstractKalmanFilter ();

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /**
     * @brief Returns the current state (estimated).
     *
     * @return The current state (estimated).
     */
    VectorXd getState () const;

    /** 
     * @brief Sets the initial state.
     *
     * @param x0 Initial state (a state is represented by the relevant
     * variables, hence a state is a vector in general).
     */
    void setInitialState (VectorXd& x0);

    /**
     * @brief Sets the initial state a zero-vector of specified size.
     *
     * When no initial state is set, there possibly is no variable or
     * parameter for the KF where the size of the state vector could
     * be extracted.
     *
     * @param n The size of the state vector.
     */
    void setSizeOfState (unsigned int n);

    /** 
     * @brief Sets the initial error covariance.
     *
     * Optional, the error covariance will stabilize quickly.  
     *
     * @param P0 Initial error covariance.
     */
    void setInitialErrorCovariance(MatrixXd& P0);

    /** 
     * @brief Sets the process noise covariance.
     *
     * Determination of process noise covariance is not that simple,
     * so its best to put here "enough" uncertainty (higher value of
     * Q), i.e. rely more on the measurements.
     *
     * @param Q Process noise covariance.
     */
    void setProcessNoiseCovariance (MatrixXd& Q);

    /** 
     * @brief Sets the measurement noise covariance.
     *
     * Can be determined by taking "offline" samples of the
     * measurements to get the variance e.g. of a sensor.
     *
     * @param R Measurement noise covariance.
     */
    void setMeasurementNoiseCovariance (MatrixXd& R);

    /**
     * @brief Sets the size of the control input vector. 
     *
     * This is the only way to initialize the control input
     * vector. When the control input is included in the state
     * transition formulas, this setter MUST be called. Otherwise an
     * exception will be thrown when applying the state transition
     * model f.
     * 
     * @param l The size of the control input vector.
     */
    void setControlInputSize (unsigned int l);

    /**
     * @brief Releases the Kalman filter if the parameters are
     * correct. 
     *
     * Throws on error. If the validation completes successfully the
     * method \ref estimate can be used.
     */
    virtual void validate (void) = 0;

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
    virtual Output estimate (Input next) = 0;
    
    /** 
     * @brief Sets the control input.
     *
     * The control input is an optional vector.
     *
     * @param u The control input.
     */
    void setControlInput (Input u);

    /**
     * @brief Returns the last estimated value.
     */
    Output getLastEstimate (void);

    /**
     * @brief Prints (debug) information of this Kalman filter
     * (current states of vectors and matrices).
     */
    virtual void serialize(std::ostream& os) const;

  protected:
    /**
     * @brief Puts the estimated state and its variance into an \c
     * Output object to match the interface \c IEstimator.
     *
     * TODO: fill jitter_ms
     */
    void updateOutput(void);
  };
}

#endif
