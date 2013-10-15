/**
 * @file 
 * @author Denise Ratasich
 * @date 14.10.2013
 *
 * @brief Implement sampling from specific distributions.
 *
 * Currently only Gaussian distributions can be choosen for process
 * and covariance noise. Hence only sampling from a Gaussian
 * distribution is implemented.
 */

#include "probability/sampling.h"

#include <stdexcept>
#include <cmath>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>

namespace probability
{
  // create the rng only once (otherwise the numbers generated are
  // always the same)
  static boost::mt19937 rng(time(NULL));

  VectorXd sampleNormalDistribution(VectorXd mean, MatrixXd covariance)
  {
    // save last covariance (so Cholesky decomposition needs not to
    // be done every time entering this function)
    static MatrixXd cov;		
    static MatrixXd T;			// transform matrix when cov is not diagonal

    // additionally needed for rng
    boost::normal_distribution<> dist(0,1);	// mean = 0, standard deviation = 1
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > random(rng, dist);

    VectorXd sample = VectorXd::Zero(mean.size());

    // check if covariance is diagonal (set diagonal to zero and
    // compare with MatrixXD::Zero)
    if (covariance.isDiagonal())
    {
      // sample each variable independently
      for (int i = 0; i < sample.size(); i++)
	sample[i] = mean[i] + random() * sqrt(covariance(i,i));
    } 
    else
    {
      // x ~ N(0,C^-1)
      // C = L*L^T
      // sample x = L^-T * z, where z ~ N(0,I)
      
      // same covariance as before? (do calculation of L only once!)
      if (cov.rows() != covariance.rows() ||
	  cov.cols() != covariance.cols() ||
	  cov != covariance)
	{
	  cov = covariance;

	  // Cholesky decomposition of inverse covariance to get L;
	  // the covariance must be a symmetric and positive definite
	  // matrix (a covariance has these features, but the user
	  // must configure it so)
	  covariance = covariance.inverse();
	  MatrixXd L = covariance.llt().matrixL();
	  
	  MatrixXd checkLL = L*L.transpose();
	  for (int r = 0; r < covariance.rows(); r++)
	    for (int c = 0; c < covariance.cols(); c++)
	      if (checkLL(r,c) - covariance(r,c)  >  0.000001)
		throw std::runtime_error("sampleNormalDistribution: Cholesky decomposition failed.");

	  T = L.inverse().transpose();
	}

      // sample z: z_i ~ N(0,1)
      for (int i = 0; i < sample.size(); i++)
	sample[i] = random();

      // transform to x (and add mean!)
      sample = mean + T * sample;
    }

    return sample;
  }

  VectorXd sampleUniformDistribution(VectorXd a, VectorXd b)
  {
    VectorXd sample = VectorXd::Zero(a.size());

    if (a.size() != b.size())
      throw std::runtime_error("sampleUniformDistribution: Bounds a and b must have equal sizes.");

    // additionally needed for rng
    boost::uniform_real<> dist(0,1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> >
      random(rng, dist);

    // sample each variable independently
    for (int i = 0; i < sample.size(); i++)
      sample[i] = a[i] + random() * (b[i]-a[i]);

    return sample;
  }
}

