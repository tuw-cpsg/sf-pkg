#include <gtest/gtest.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

// testing following API
#include "estimation/ParticleFilterSIR.h"
#include "probability/pdfs.h"
#include "probability/sampling.h"

using namespace std;
using namespace estimation;
using namespace probability;

namespace logTest
{
  // -----------------------------------------
  // logging for nice pictures
  // -----------------------------------------
  void f_log(VectorXd& x, const VectorXd& u)
  {
    x[0] = x[0];
  }
  void h_log(VectorXd& z, const VectorXd& x)
  {
    z[0] = x[0];
  }

  TEST(LogTest, logEstimationOfConstantNoisySignal)
  {
    // example 1: constant signal, but gaussian noisy with mean 0,
    // variance 1

    // create and open log file(s): signal, particles
    ofstream logSignal;
    logSignal.open("logEstimationOfConstantNoisySignal_signal.dat");
    ofstream logParticles;
    logParticles.open("logEstimationOfConstantNoisySignal_particles.dat");
    ofstream logWeights;
    logWeights.open("logEstimationOfConstantNoisySignal_weights.dat");
    ofstream logNeff;
    logNeff.open("logEstimationOfConstantNoisySignal_Neff.dat");
    ofstream logSignalEstimated;
    logSignalEstimated.open("logEstimationOfConstantNoisySignal_signalEstimated.dat");

    // init particle filter
    ParticleFilterSIR pf(1000);

    pf.setStateTransitionModel(f_log);
    pf.setObservationModel(h_log);
    MatrixXd Q(1,1); Q << 0.01;
    pf.setProcessNoiseCovariance(Q);
    MatrixXd R(1,1); R << 1;
    pf.setMeasurementNoiseCovariance(R);
    VectorXd x0(1); x0 << 1;
    pf.setInitialState(x0);

    pf.validate();
    
    // measurements
    VectorXd mean = VectorXd::Ones(1);		// mean = 1
    MatrixXd var = MatrixXd::Identity(1,1);	// variance = 1

    Input signal(InputValue(1));
    Output signalEstimated;

    // start filtering and log
    for (int i = 0; i < 40; i++)
    {
      // create new measurement
      double sample = sampleNormalDistribution(mean,var)[0];
      signal[0].setValue(sample);

      // estimate
      signalEstimated = pf.estimate(signal);

      // log
      logSignal << (i+1) << " " 
		<< sample << endl;
      pf.log(logParticles, ParticleFilterSIR::PARTICLES, 0);
      pf.log(logWeights, ParticleFilterSIR::WEIGHTS, 0);
      logNeff << (i+1) << " ";
      pf.log(logNeff, ParticleFilterSIR::NEFF, 0);
      logSignalEstimated << (i+1) << " "
			 << signalEstimated[0].getValue() << " "
			 << sqrt(signalEstimated[0].getVariance())
			 << endl;
    }

    logSignal.close();
    logParticles.close();
    logWeights.close();
    logNeff.close();
    logSignalEstimated.close();
  }

  TEST(LogTest, logSampleNormalDistribution)
  {
    int N = 10000;
    VectorXd mean = VectorXd::Zero(1);
    MatrixXd cov = MatrixXd::Identity(1,1);

    ofstream logFile;
    logFile.open("logSampleNormalDistribution.dat");
    for (int i = 0; i < N; i++)
      logFile << sampleNormalDistribution(mean, cov) 
    	      << endl;

    logFile.close();
  }

  TEST(LogTest, logSampleUniformDistribution)
  {
    int N = 10000;
    VectorXd a = VectorXd::Zero(1);
    VectorXd b = VectorXd::Ones(1);

    ofstream logFile;
    logFile.open("logSampleUniformDistribution.dat");
    for (int i = 0; i < N; i++)
      logFile << sampleUniformDistribution(a, b) 
    	      << endl;

    logFile.close();
  }

  TEST(LogTest, logPdfNormalDistribution)
  {
    VectorXd x = VectorXd::Zero(1);
    VectorXd mean = VectorXd::Zero(1);
    MatrixXd cov = MatrixXd::Identity(1,1);

    ofstream logFile;
    logFile.open("logPdfNormalDistribution.dat");
    for (float i = -2.0; i < 2.0; i=i+0.1)
    {
      x[0] = i;
      logFile << i << " "
	      << pdfNormalDistribution(x, mean, cov) 
    	      << endl;
    }
    logFile.close();
  }
}
