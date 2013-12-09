/**
 * @file 
 * @author Denise Ratasich
 * @date 11.10.2013
 *
 * @brief Implementation of the SIR particle filter.
 */

#include <ctime>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
// uniform_real gets depricated with newer versions, change to
// uniform_real_distribution with newer version of boost! (see also
// ParticleFilterSIR::resample)
//#include <boost/random/uniform_real_distribution.hpp>

#include "estimation/ParticleFilterSIR.h"
#include "probability/pdfs.h"
#include "probability/sampling.h"

namespace estimation 
{
  ParticleFilterSIR::ParticleFilterSIR (unsigned int N) 
    : AbstractParticleFilter(N)
  {
    rng.seed((unsigned int)clock());
  }

  ParticleFilterSIR::~ParticleFilterSIR () 
  {
    // nothing to do
  }
  
  void ParticleFilterSIR::log(std::ostream& os, Log type, int index) const
  {
    switch (type)
    {
    case PARTICLES:
      for (int i = 0; i < particles.size(); i++)
	os << particles[i][index] << " ";
      os << std::endl;
      break;
    case WEIGHTS:
      for (int i = 0; i < particles.size(); i++)
	os << weights[i] << " ";
      os << std::endl;
      break;
    case NEFF:
      os << Neff << std::endl;
      break;
    }
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  void ParticleFilterSIR::setProcessNoiseCovariance (MatrixXd& Q)
  {
    this->Q = Q;
    validated = false;
  }

  void ParticleFilterSIR::setMeasurementNoiseCovariance (MatrixXd& R)
  {
    this->R = R;
    validated = false;
  }

  void ParticleFilterSIR::validate (void)
  {
    try 
    {
      // check for minimal initialization -----------------------
      if (!f)			// callback empty
	throw std::runtime_error("State transition model missing.");
      if (!h)			// callback empty
	throw std::runtime_error("Observation model missing.");
      if (Q.rows() == 0)
	throw std::runtime_error("Process noise covariance missing.");
      if (R.rows() == 0)
	throw std::runtime_error("Measurement noise covariance missing.");
      if (particles[0].size() == 0)
	throw std::runtime_error("Particles not initialized.");      

      // take sizes from required parameters
      int n = particles[0].size();
      int m = R.rows();

      // create other parameters if missing ---------------------
      // none

      // check appropriate sizes of matrices and vectors --------
      if (Q.rows() != n  ||  Q.cols() != n)
	throw std::runtime_error("Process noise covariance has invalid size.");
      if (R.rows() != R.cols())
	throw std::runtime_error("Measurement noise covariance must be a square matrix.");

      // validation finished successfully -----------------------
      validated = true;

      // further things to initialize ---------------------------
      // create output state
      if (out.size() != n) {	// not initialized till now or invalid size from reinit
	out.clear();
	for (int i = 0; i < n; i++)
	  out.add(OutputValue());
      }
    } 
    catch(std::exception& e) 
    {
      std::string additionalInfo = "ParticleFilterSIR: Validation failed. ";
      throw estimator_error(additionalInfo + e.what());
    }
  }

  // -----------------------------------------
  // Overrides of ParticleFilterSIR's IEstimator implementation
  // -----------------------------------------

  void ParticleFilterSIR::serialize(std::ostream& os) const
  {
    os << "SIR Particle Filter";
  }

  // -----------------------------------------
  // Particle Filtering
  // -----------------------------------------
  void ParticleFilterSIR::sample (void)
  {    
    // estimate next state of particles
    for (int i = 0; i < particles.size(); i++)
    {      
      // sample from process noise: zero mean gaussian with covariance Q
      VectorXd w = probability::sampleNormalDistribution(VectorXd::Zero(Q.rows()), Q);
      
      f(particles[i], u);		// time update
      particles[i] = particles[i] + w;  // add noise
    }
  }

  void ParticleFilterSIR::weight (Input measurements)
  {
    double sumWeights = 0, sumWeightsSquare = 0;
   
    // measurement vector z
    VectorXd z(R.rows());

    if (measurements.size() != z.size())
      throw std::runtime_error("Number of measurements invalid.");

    // calculate weight
    for (int i = 0; i < particles.size(); i++)
    {
      // weight = likelihood of the measurement
      // estimate expected measurement when in state i
      VectorXd z_expected = VectorXd::Zero(z.size());
      h(z_expected, particles[i]);

      // get probability of the measurement (mean = z_expected,
      // covariance = measurement noise covariance)
      prepareMeasurements(z, measurements, z_expected);	// fills z
      double weight = probability::pdfNormalDistribution(z, z_expected, R);
      
      weights[i] = weights[i] * weight;	// set weight (consider old weight!)
      sumWeights += weights[i];		// add to sum for normalization
    }

    // normalize weights
    for (int i = 0; i < particles.size(); i++)
    {
      weights[i] /= sumWeights;

      // accumulate for Neff-calculation
      sumWeightsSquare += weights[i] * weights[i];
    }

    // evaluate effective number of particles
    Neff = 1 / sumWeightsSquare;
  }

  void ParticleFilterSIR::resample (void)
  {
    int N = particles.size();

    if (Neff > 0.8*N)
      return;

    // copy current particles to the buffer 'partices_old' where the
    // new ones will be chosen from; the array 'particles' will be
    // filled with these drawn ones
    std::vector<VectorXd> particles_old(particles);

    // 1. generate CDF
    std::vector<double> cdf(N);
    cdf[0] = weights[0];
    for (int i = 1; i < N; i++)
      cdf[i] = cdf[i-1] + weights[i];
    
    // 2. draw a starting point
    boost::uniform_real<> dist(0, 1.0 / N);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> >
      random(rng, dist);
    double u0 = random();	// u0 is then betw 0..1/N
    int i = 0;

    for (int j = 0; j < N; j++)
    {
      // 3. move along the CDF
      double uj = u0 + ((double)j)/N;

      // check where the random number in the CDF fits -> this is the
      // sample to choose; a sample with higher weight has a higher
      // span in the CDF, hence will be chosen more often
      while (uj > cdf[i])
      	i++;

      particles[j] = particles_old[i];
      weights[j] = 1.0 / N;
    }
  }
}
