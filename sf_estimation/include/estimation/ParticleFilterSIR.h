/**
 * @file 
 * @author Denise Ratasich
 * @date 11.10.2013
 *
 * @brief Header file of the particle filter.
 */

#ifndef __ESTIMATION_PARTICLE_FILTER_SIR_H__
#define __ESTIMATION_PARTICLE_FILTER_SIR_H__

#include <vector>
#include <ostream>

#include "estimation/AbstractParticleFilter.h"
#include "estimation/Input.h"
#include "estimation/Output.h"

#include <boost/random/mersenne_twister.hpp>
#include <Eigen/Dense>
using namespace Eigen;

namespace estimation 
{
  /**
   * @brief SIR Particle Filter.
   *
   * This concrete particle filter uses $p(x_k|x_{k-1})$ as the
   * importance density.
   *
   * Stored parameters:
   *
   * \note The particle filter needs to be validated, i.e. the method
   * \ref validate must be called to release the operation. Changing a
   * parameter needs revalidation.
   */
  class ParticleFilterSIR : public AbstractParticleFilter
  {
  private:
    /** @brief Random number generator. */
    boost::mt19937 rng;

    // -----------------------------------------
    // parameters
    // -----------------------------------------

    /** @brief Process noise covariance. Size: n x n. */
    MatrixXd Q;
    /** @brief Measurement noise covariance. Size: m x m. */
    MatrixXd R;

    /** Effective number of particles, i.e. measure of degeneracy. */
    double Neff;

  public: 
    /**
     * @brief Constructor setting the type of the particle filter.
     *
     * @param N The number of particles.
     */
    ParticleFilterSIR (unsigned int N);

    /**
     * @brief Destructor of this class.
     */
    ~ParticleFilterSIR ();

    enum Log 
    {
      PARTICLES,
      WEIGHTS,
      NEFF
    };

    /**
     * @brief Prints log information: first state variables of all
     * particles, i.e. particles[i][0].
     */
    void log(std::ostream& os, Log type, int index) const;

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
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
     * @brief Releases the particle filter if the parameters are
     * correct. 
     *
     * Throws on error. If the validation completes successfully the
     * method \ref estimate can be used.
     */
    virtual void validate (void);

    // -----------------------------------------
    // Overrides of AbstractParticleFilter's IEstimator implementation
    // -----------------------------------------
    /**
     * @brief Prints (debug) information of this particle filter.
     */
    virtual void serialize(std::ostream& os) const;

  protected:
    // -----------------------------------------
    // Particle Filtering
    // -----------------------------------------
    /**
     * @brief Samples the particles according the importance density.
     * 
     * The state transition model f is applied to every particle. A
     * noise value is sampled and added to each particle.
     * 
     * Every particle should have the same weight, hence resampling
     * before this step is necessary.
     */
    virtual void sample (void);

    /**
     * @brief Assigns a (normalized) weight to each particle.
     *
     * Calculates the conditional probability of an observed
     * measurement z under x (\f$p(z|x)\f$).
     */
    virtual void weight (VectorXd z);

    /**
     * @brief Eliminates particles with negligible weights.
     *
     * Resampling is done in every cycle. After these step, all
     * weights equal to \f$1/N\f$ (N..number of particles).
     */
    virtual void resample (void);
  };

}

#endif
