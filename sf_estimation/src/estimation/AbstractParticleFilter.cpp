/**
 * @file 
 * @author Denise Ratasich
 * @date 11.10.2013
 *
 * @brief Implementation of a particle filter.
 */

#include "estimation/AbstractParticleFilter.h"
#include "probability/sampling.h"

namespace estimation 
{
  AbstractParticleFilter::AbstractParticleFilter (unsigned int N)
  {
    // set number of particles
    particles.resize(N);
    weights.resize(N);

    validated = false;
  }

  AbstractParticleFilter::~AbstractParticleFilter () 
  {
    // nothing to do
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  void AbstractParticleFilter::setInitialState (VectorXd& x0)
  {
    int N = particles.size();
    for (int i = 0; i < N; i++) 
    {
      particles[i] = x0;
      weights[i] = 1.0 / N;
    }

    validated = false;
  }

  void AbstractParticleFilter::initializeParticles (VectorXd& lo, VectorXd& up)
  {
    int N = particles.size();

    // assign each particle a random state
    for (int i = 0; i < N; i++) 
    {
      particles[i] = probability::sampleUniformDistribution(lo, up);
      weights[i] = 1.0 / N;
    }
  }

  void AbstractParticleFilter::setStateTransitionModel (func_f f)
  {
    this->f = f;
    validated = false;
  }

  void AbstractParticleFilter::setObservationModel (func_h h)
  {
    this->h = h;
    validated = false;
  }

  void AbstractParticleFilter::setControlInputSize (unsigned int l)
  {
    this->u = VectorXd::Zero(l);
    validated = false;
  }

  // -----------------------------------------
  // IEstimator implementation
  // -----------------------------------------
  Output AbstractParticleFilter::estimate (Input next)
  {
    try 
    {
      if (!validated)
	throw std::runtime_error("Not yet validated!");
    
      // Particle Filtering ----------------------------------------
      sample();		// samples from the importance density
      weight(next);	// weights the samples, i.e. particles
      resample();	// eliminates particles with negligible weights
      // -----------------------------------------------------------

      // insert new state and variance into the Output object
      updateOutput();
      return out;
    } 
    catch(std::exception& e) 
    {
      std::string additionalInfo = "AbstractParticleFilter: Estimation failed.";
      throw estimator_error(additionalInfo + e.what());
    }
  }

  void AbstractParticleFilter::setControlInput (Input u)
  {
    // The parameter u holds also timestamps of the control input
    // variables. For simplicity u is copied without checking
    // timestamps. When an estimation algorithm needs information
    // about the actuality of the control input, the variable type
    // should be changed to Input instead of a simple VectorXd.

    // It is assumed that u is already initialized (size of vector u
    // != 0). However, check (at least) if the sizes fit.
    if (this->u.size() != u.size())
      throw std::length_error("AbstractParticleFilter: Setting control input failed. Control input has invalid size."); 

    for (int i = 0; i < u.size(); i++)
      this->u[i] = u[i].getValue();    
  }
  
  Output AbstractParticleFilter::getLastEstimate (void)
  {
    return out;
  }

  void AbstractParticleFilter::serialize(std::ostream& os) const
  {
    os << "ParticleFilter" << std::endl
       << "control input (u) = " << this->u << std::endl;
  }

  // -----------------------------------------
  // protected (helper) functions
  // -----------------------------------------
  void AbstractParticleFilter::prepareMeasurements(VectorXd& z, Input& in, VectorXd& z_expected)
  {
    // evaluate 'z', i.e. fill with in-values or z_expected-values
    // when in-value is missing
    for (int i = 0; i < z.size(); i++)
      if (in[i].isMissing())
	z[i] = z_expected[i];
      else
	z[i] = in[i].getValue();
  }

  void AbstractParticleFilter::updateOutput(void)
  {
    int N = particles.size();		// number of particles
    int n = particles[0].size();	// state size

    // calculate the mean and covariance out of the particles which
    // represent the pdf of the current state

    // // mean: sum over i (p_i * x_i)
    VectorXd mean = VectorXd::Zero(n);
    for (int i = 0; i < N; i++)
      mean = mean + weights[i] * particles[i];

    // // variance: sum over i (p_i * (x[i] - mean)^2)
    MatrixXd variance = MatrixXd::Zero(n,n);
    for (int i = 0; i < N; i++)
    {
      VectorXd x = particles[i] - mean;
      variance = variance + weights[i] * (x * x.transpose());
    }

    // fill output with state and variance
    for (int i = 0; i < out.size(); i++) {
      out[i].setValue(mean[i]);
      out[i].setVariance(variance(i,i));
      // jitter is almost zero (updateOutput is called from the
      // estimate method), hence not set explicitly (jitter_ms is
      // initialized to 0 with setValue).
    }
  }
}
