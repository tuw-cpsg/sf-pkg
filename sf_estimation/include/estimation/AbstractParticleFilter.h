/**
 * @file 
 * @author Denise Ratasich
 * @date 11.10.2013
 *
 * @brief Header file of the particle filter.
 */

#ifndef __ESTIMATION_ABSTRACT_PARTICLE_FILTER_H__
#define __ESTIMATION_ABSTRACT_PARTICLE_FILTER_H__

#include <vector>
#include <ostream>

#include "estimation/IEstimator.h"
#include "estimation/Input.h"
#include "estimation/Output.h"

#include <boost/random.hpp>
#include <Eigen/Dense>
using namespace Eigen;

namespace estimation 
{
  /**
   * @brief Abstract Particle Filter.
   *
   * A particle filter uses particles to estimate the state's
   * probability density function (pdf). The 3 main steps are:
   * - sample a state = particle (regarding the state transition
   *   system and process noise)
   * - assign a weight to each particle (according the likelihood of
   *   the occured measurement at that specific state, the particle
   *   represents)
   * - resample to avoid degeneracy of particles
   *
   * This class implements only the generic parts of the algorithm and
   * handles parameters which are required in all particle
   * filters. Sampling, weighting and resampling are implemented in
   * the concrete particle filters.
   *
   * Stored parameters:
   * - state transition model
   * - observation or measurement model
   *
   * \note The particle filter needs to be validated, i.e. the method
   * \ref validate must be called to release the operation. Changing a
   * parameter needs revalidation.
   */
  class AbstractParticleFilter : public IEstimator
  {
  public:    
    /** 
     * @brief Pointer to a function representing the state transition
     * model.
     *
     * @param x The state x at time k-1 (input) used to calculate the
     * estimated (a priori) state vector at time k (output).
     * @param[in] The control input at time k-1.
     */
    typedef void (*func_f)(VectorXd& x, const VectorXd& u);
    /** 
     * @brief Pointer to a function representing the observation
     * model. 
     * 
     * @param[out] z The estimated measurement vector.
     * @param[in] x The a priori state vector.
     */
    typedef void (*func_h)(VectorXd& z, const VectorXd& x);

  protected:
    /** @brief Flag indicating if the parameters were checked and are
     * OK.  
     *
     * \sa function \c validate 
     * \sa for more on parameters see \ref particlefilter
     */
    bool validated;

    /** @brief Output of this estimation method (state and
     * variance). Size: n. */
    Output out;

    /** @brief The particles representing the pdf of the current
     * state. */
    std::vector<VectorXd> particles;
    /** @brief The weights of the particles. Should always be in range
     * of 0 and 1. */
    std::vector<double> weights;

    // -----------------------------------------
    // parameters
    // -----------------------------------------
    /** 
     * @brief Probably nonlinear function which represents the state
     * transition model, calculates the estimated next state.
     */
    func_f f;
    /** 
     * @brief Probably nonlinear function which represents the
     * observation model, calculates the estimated measurement vector.
     */
    func_h h;
    /** @brief Control input. Size: l. */
    VectorXd u;

  public: 
    /**
     * @brief Constructor setting the type of the particle filter.
     *
     * @param N The number of particles.
     */
    AbstractParticleFilter (unsigned int N);

    /**
     * @brief Destructor of this class.
     */
    ~AbstractParticleFilter ();

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /**
     * @brief Sets the initial state of the filter. 
     *
     * Calling this method means the initial state is known, hence all
     * particles are initialized with the given state in x0 and
     * weighted with \f$1/N\f$. This represents indeed the initial
     * probability, which is 1 at \f$x = x_0\f$ and 0 everywhere else
     * (\f$x \ne x_0\f$).
     *
     * When the initial state is unknown use the function
     * initializeParticles(). The particles must be initialized
     * somehow, otherwise validation will fail.
     */
    void setInitialState (VectorXd& x0);

    /**
     * @brief Initializes the particles with random states.
     *
     * This function MUST be called when the initial state is
     * unknown. Otherwise validation will fail.
     *
     * Randomly generates particles (distributed uniformly),
     * i.e. possible states, hence ranges of the state variables must
     * be given. The state size, i.e. the number of state variables is
     * extracted out of these vectors.
     *
     * @param lo Lower bound of the state range.
     * @param up Upper bound of the state range.
     */
    void initializeParticles (VectorXd& lo, VectorXd& up);

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
     * @brief Sets the size of the control input vector. 
     *
     * This is the only way to initialize the control input
     * vector. When the control input is included in the state
     * transition formulas, this setter MUST be called when the
     * control input is used in f. Otherwise an exception will be
     * thrown when applying the state transition model f.
     * 
     * @param l The size of the control input vector.
     */
    void setControlInputSize (unsigned int l);

    /**
     * @brief Releases the particle filter if the parameters are
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
     * When calling without prior validation this method throws an
     * exception.
     */
    virtual Output estimate (Input next);
    
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
     * @brief Prints (debug) information of this particle filter.
     */
    virtual void serialize(std::ostream& os) const;

  protected:
    // -----------------------------------------
    // Particle Filtering
    // -----------------------------------------
    
    /**
     * @brief Samples the particles according the importance density.
     */
    virtual void sample (void) = 0;

    /**
     * @brief Assigns a (normalized) weight to each particle.
     */
    virtual void weight (Input measurements) = 0;

    /**
     * @brief Eliminates particles with negligible weights.
     */
    virtual void resample (void) = 0;

    /**
     * @brief Fills the measurement vector z.
     *
     * Missing values of Input in are replaced by the expected ones
     * (already available in z_expected), i.e. on state variables
     * depending on missing measurements only a time update will be
     * applied (the term K*(z-z_expected) is dropped).
     *
     * @param z The measurement vector.
     * @param in The InputValue(s), i.e. the measurements (this Input
     * may also contain uninitialized InputValues - these will be
     * interpreted as missing measurements).
     * @param z_expected The expected measurements regarding the
     * current a priori state.
     */
    void prepareMeasurements (VectorXd& z, Input& in, VectorXd& z_expected);

    /**
     * @brief Puts the estimated state and its variance into an \c
     * Output object to match the interface \c IEstimator.
     */
    void updateOutput (void);
  };

}

#endif
