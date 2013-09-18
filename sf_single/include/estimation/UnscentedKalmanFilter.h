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

#include "estimation/IEstimator.h"
#include "estimation/Input.h"
#include "estimation/Output.h"
#include "estimation/ITransformer.h"

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
  class UnscentedKalmanFilter : public IEstimator, public ITransformer
  {
  public:
    /** Pointer to a function representing the state transition
     * model. */
    typedef void (*func_f)(VectorXd& x, const VectorXd& u);
    /** Pointer to a function representing the observation model. */
    typedef void (*func_h)(VectorXd& z, const VectorXd& x);

  private:
    enum ut_functionType {
      STATE_TRANSITION_MODEL,
      OBSERVATION_MODEL
    };
    
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

    /** @brief When true the Unscented Transform uses the state
     * transition model, otherwise the measurement model is used. */
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
    /** @brief Jacobian of f with respect to x_{k-1}, derivation of
     * the system model to x. Size: n x n.*/
    MatrixXd A;
    /** @brief Control input. Size: l. */
    VectorXd u;
    /** @brief Process noise covariance. Size: n x n. */
    MatrixXd Q;

    /** 
     * @brief Probably nonlinear function which represents the
     * observation model, calculates the estimated measurement vector.
     *
     * @param[out] z The estimated measurement vector.
     * @param[in] x The a priori state vector.
     */
    func_h h;
    /** @brief Jacobian of h with respect to a priori x, derivation of
     * the observation model. Size: m x n. */
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
    UnscentedKalmanFilter ();

    /**
     * @brief Destructor of this class.
     */
    ~UnscentedKalmanFilter ();

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
     * @brief Sets the initial state a zero-vector of specified size.
     *
     * When no initial state is set, validation cannot be done. There
     * is no variable or parameter for the EKF where the size of the
     * state vector could be extracted.
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
    void setInitialErrorCovariance(std::vector< std::vector<double> >& P0);

    /** 
     * @brief Sets the state transition model, a callback function
     * which calculates the next state.
     *
     * @param f The state transition model.
     */
    void setStateTransitionModel (func_f f);
    
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
     * @brief Sets the observation model, a callback function which
     * calculates the estimated measurement vector according to the
     * current state.
     *
     * @param h Observation model.
     */
    void setObservationModel (func_h h);

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
     * @brief Returns the last estimated value.
     */
    Output getLastEstimate (void);

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
     * @return The expected state or measurement according which model
     * should be used.
     */
    VectorXd transform (const VectorXd& x);

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
     * Output object to match the interface \c IEstimator.
     *
     * TODO: fill jitter_ms
     */
    void updateOutput(void);
  };

}

#endif
