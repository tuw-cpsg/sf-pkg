/**
 * @file 
 * @author Denise Ratasich
 * @date 03.09.2013
 *
 * @brief Implementation of the EstimatorFactory.
 */

#include "estimation/EstimatorFactory.h"

namespace estimation 
{
  void EstimatorFactory::addParam(std::string key, boost::any value)
  {
    // when key already exists the value will be overwritten
    params[key] = value;
  }

  void EstimatorFactory::reset(void)
  {
    params.clear();
  }

  estimation::IEstimator* EstimatorFactory::create (void)
  {
    if (params.count("method") == 0)
      throw factory_error("Method not specified.");

    try {
      std::string method = boost::any_cast<std::string>(params["method"]);

      if (method.compare("MovingMedian") == 0)
      {
	estimation::MovingMedian* mm = new estimation::MovingMedian();
	initMovingMedian(*mm);
	return mm;
      }

      if (method.compare("MovingAverage") == 0)
      {
	estimation::MovingAverage* ma = new estimation::MovingAverage();
	initMovingAverage(*ma);
	return ma;
      }

      if (method.compare("KalmanFilter") == 0)
      {
	estimation::KalmanFilter* kf = new estimation::KalmanFilter();
	initKalmanFilter(*kf);
	return kf;
      }

      if (method.compare("ExtendedKalmanFilter") == 0)
      {
	estimation::ExtendedKalmanFilter* ekf = new estimation::ExtendedKalmanFilter();
	initExtendedKalmanFilter(*ekf);
	return ekf;
      }

      if (method.compare("UnscentedKalmanFilter") == 0)
      {
	estimation::UnscentedKalmanFilter* ukf = new estimation::UnscentedKalmanFilter();
	initUnscentedKalmanFilter(*ukf);
	return ukf;
      }

      if (method.compare("ParticleFilterSIR") == 0)
      {
	estimation::ParticleFilterSIR* pf;

	// create particle filter -----
	if (params.count("number-of-particles")) {
	  int N = boost::any_cast<int>(params["number-of-particles"]);
	  pf = new ParticleFilterSIR(N);
	} else {
	  pf = new ParticleFilterSIR(1000);
	}

	initParticleFilterSIR(*pf);
	return pf;
      }

      if (method.compare("ConfidenceWeightedAveraging") == 0)
      {
	estimation::ConfidenceWeightedAveraging* cwa = new estimation::ConfidenceWeightedAveraging();
	initConfidenceWeightedAveraging(*cwa);
	return cwa;
      }
    } catch(std::exception& e) {
      throw factory_error(std::string("Configuration failed. ") + e.what());
    }

    throw factory_error("Configuration failed. Unknown method.");
  }

  // -----------------------------------------
  // initialization of the specific estimator
  // -----------------------------------------

  void EstimatorFactory::initMovingMedian(estimation::MovingMedian& mm)
  {
    // required: none
    // optional: window-size

    try {
      if (params.count("window-size")) {
	int ws = boost::any_cast<int>(params["window-size"]);
	mm.setWindowSize(ws);
      }
    } catch(std::exception& e) {
      std::string additionalInfo = "Initializing MovingMedian failed. ";
      throw factory_error(additionalInfo + e.what());
    }
  }

  void EstimatorFactory::initMovingAverage(estimation::MovingAverage& ma)
  {
    // required: none
    // optional: window-size, weighting-coefficients

    try {
      int ws = ma.getWindowSize();	// default window size
      if (params.count("window-size")) {
	ws = boost::any_cast<int>(params["window-size"]);
	ma.setWindowSize(ws);
      }

      if (params.count("weighting-coefficients")) {
	VectorXd vectorWc = boost::any_cast<VectorXd>(params["weighting-coefficients"]);
	std::vector<double> wc;
	for (int i = 0; i < vectorWc.size(); i++)
	  wc.push_back(vectorWc[i]);
  
	if (wc.size() == ws) {
	  // only b_k coefficients are passed
	  ma.setWeightingCoefficientsIn(&wc[0], ws);
	} else if (wc.size() == 2*ws-1) {
	  // a_k coefficients also passed through arguments
	  // first (ws-1) numbers
	  ma.setWeightingCoefficientsOut(&wc[0], ws-1);
	  ma.setWeightingCoefficientsIn(&wc[ws-1], ws);
	} else {
	  throw std::length_error("Configuration failed. "
				  "Invalid number of "
				  "weighting-coefficients "
				  "(must equal "
				  "1x or 2x-1 window size).");
	}
      }
    } catch(std::exception& e) {
      std::string additionalInfo = "Initializing MovingAverage failed. ";
      throw factory_error(additionalInfo + e.what());
    }
  }

  void EstimatorFactory::initKalmanFilter(estimation::KalmanFilter& kf)
  {
    // required: state-transition-model, process-noise-covariance,
    //   observation-model, measurement-noise-covariance
    // optional: control-input-model, control-input, initial-state,
    //   initial-error-covariance

    // required -----
    try {
      if (params.count("state-transition-model")) {
	MatrixXd stm = boost::any_cast<MatrixXd>(params["state-transition-model"]);
	kf.setStateTransitionModel(stm);
      } else
	throw factory_error("State transition model missing.");

      if (params.count("process-noise-covariance")) {
	MatrixXd pnc = boost::any_cast<MatrixXd>(params["process-noise-covariance"]);
	kf.setProcessNoiseCovariance(pnc);
      } else
	throw factory_error("Process noise covariance missing.");

      if (params.count("observation-model")) {
	MatrixXd om = boost::any_cast<MatrixXd>(params["observation-model"]);
	kf.setObservationModel(om);
      } else
	throw factory_error("Observation model missing.");

      if (params.count("measurement-noise-covariance")) {
	MatrixXd mnc =  boost::any_cast<MatrixXd>(params["measurement-noise-covariance"]);
	kf.setMeasurementNoiseCovariance(mnc);
      } else
	throw factory_error("Measurement noise covariance missing.");

      // optional -----

      if (params.count("control-input-model")) {
	MatrixXd cim = boost::any_cast<MatrixXd>(params["control-input-model"]);
	kf.setControlInputModel(cim);
      }

      if (params.count("control-input-size")) {
	int cis = boost::any_cast<int>(params["control-input-size"]);
	kf.setControlInputSize(cis);
      }

      if (params.count("initial-state")) {
	VectorXd is = boost::any_cast<VectorXd>(params["initial-state"]);
	kf.setInitialState(is);
      }

      if (params.count("initial-error-covariance")) {
	MatrixXd iec = boost::any_cast<MatrixXd>(params["initial-error-covariance"]);
	kf.setInitialErrorCovariance(iec);
      }

      kf.validate();	// throws on error
    } catch(std::exception& e) {
      std::string additionalInfo = "Initializing KalmanFilter failed. ";
      throw factory_error(additionalInfo + e.what());
    }
  }

  void EstimatorFactory::initExtendedKalmanFilter(estimation::ExtendedKalmanFilter& ekf)
  {
    // required: state-transition-model,
    //   state-transition-model-jacobian, process-noise-covariance,
    //   observation-model, observation-model-jacobian,
    //   measurement-noise-covariance

    // optional: control-input, initial-state,
    //   initial-error-covariance

    // required -----
    try {
      if (params.count("state-transition-model")) {
	func_f stm = boost::any_cast<func_f>(params["state-transition-model"]);
	ekf.setStateTransitionModel(stm);
      } else
	throw factory_error("State transition model missing.");

      if (params.count("state-transition-model-jacobian")) {
	func_df stmj = boost::any_cast<func_df>(params["state-transition-model-jacobian"]);
	ekf.setJacobianOfStateTransitionModel(stmj);
      } else
	throw factory_error("Jacobian of state transition model missing.");

      if (params.count("process-noise-covariance")) {
	MatrixXd pnc = boost::any_cast<MatrixXd>(params["process-noise-covariance"]);
	ekf.setProcessNoiseCovariance(pnc);
      } else
	throw factory_error("Process noise covariance missing.");

      if (params.count("observation-model")) {
	func_h om = boost::any_cast<func_h>(params["observation-model"]);
	ekf.setObservationModel(om);
      } else
	throw factory_error("Observation model missing.");

      if (params.count("observation-model-jacobian")) {
	func_dh omj = boost::any_cast<func_dh>(params["observation-model-jacobian"]);
	ekf.setJacobianOfObservationModel(omj);
      } else
	throw factory_error("Jacobian of observation model missing.");

      if (params.count("measurement-noise-covariance")) {
	MatrixXd mnc =  boost::any_cast<MatrixXd>(params["measurement-noise-covariance"]);
	ekf.setMeasurementNoiseCovariance(mnc);
      } else
	throw factory_error("Measurement noise covariance missing.");

      // optional -----

      if (params.count("control-input-size")) {
      	int cis = boost::any_cast<int>(params["control-input-size"]);
      	ekf.setControlInputSize(cis);
      }

      if (params.count("initial-state")) {
	VectorXd is = boost::any_cast<VectorXd>(params["initial-state"]);
	ekf.setInitialState(is);
      }

      if (params.count("initial-error-covariance")) {
	MatrixXd iec = boost::any_cast<MatrixXd>(params["initial-error-covariance"]);
	ekf.setInitialErrorCovariance(iec);
      }

      ekf.validate();	// throws on error
    } catch(std::exception& e) {
      std::string additionalInfo = "Initializing ExtendedKalmanFilter failed. ";
      throw factory_error(additionalInfo + e.what());
    }
  }

  void EstimatorFactory::initUnscentedKalmanFilter(UnscentedKalmanFilter& ukf)
  {    
    // required: state-transition-model, process-noise-covariance,
    //   observation-model, measurement-noise-covariance

    // optional: control-input, initial-state,
    //   initial-error-covariance

    try {

      // required -----

      if (params.count("state-transition-model")) {
	func_f stm = boost::any_cast<func_f>(params["state-transition-model"]);
	ukf.setStateTransitionModel(stm);
      } else
	throw factory_error("State transition model missing.");

      if (params.count("process-noise-covariance")) {
	MatrixXd pnc = boost::any_cast<MatrixXd>(params["process-noise-covariance"]);
	ukf.setProcessNoiseCovariance(pnc);
      } else
	throw factory_error("Process noise covariance missing.");

      if (params.count("observation-model")) {
	func_h om = boost::any_cast<func_h>(params["observation-model"]);
	ukf.setObservationModel(om);
      } else
	throw factory_error("Observation model missing.");

      if (params.count("measurement-noise-covariance")) {
	MatrixXd mnc =  boost::any_cast<MatrixXd>(params["measurement-noise-covariance"]);
	ukf.setMeasurementNoiseCovariance(mnc);
      } else
	throw factory_error("Measurement noise covariance missing.");

      // optional -----

      if (params.count("control-input-size")) {
	int cis = boost::any_cast<int>(params["control-input-size"]);
	ukf.setControlInputSize(cis);
      }

      if (params.count("initial-state")) {
	VectorXd is = boost::any_cast<VectorXd>(params["initial-state"]);
	ukf.setInitialState(is);
      }

      if (params.count("initial-error-covariance")) {
	MatrixXd iec = boost::any_cast<MatrixXd>(params["initial-error-covariance"]);
	ukf.setInitialErrorCovariance(iec);
      }

      ukf.validate();	// throws on error
    } catch(std::exception& e) {
      std::string additionalInfo = "Initializing UnscentedKalmanFilter failed. ";
      throw factory_error(additionalInfo + e.what());
    }
  }

  void EstimatorFactory::initParticleFilterSIR(ParticleFilterSIR& pf)
  {    
    // required: state-transition-model, process-noise-covariance,
    //   observation-model, measurement-noise-covariance,
    //   initial-state / state-bounds

    // optional: number-of-particles, control-input-size

    try {

      // required -----

      if (params.count("state-transition-model")) {
      	func_f stm = boost::any_cast<func_f>(params["state-transition-model"]);
      	pf.setStateTransitionModel(stm);
      } else
      	throw factory_error("State transition model missing.");

      if (params.count("process-noise-covariance")) {
      	MatrixXd pnc = boost::any_cast<MatrixXd>(params["process-noise-covariance"]);
      	pf.setProcessNoiseCovariance(pnc);
      } else
      	throw factory_error("Process noise covariance missing.");

      if (params.count("observation-model")) {
      	func_h om = boost::any_cast<func_h>(params["observation-model"]);
      	pf.setObservationModel(om);
      } else
      	throw factory_error("Observation model missing.");

      if (params.count("measurement-noise-covariance")) {
      	MatrixXd mnc = boost::any_cast<MatrixXd>(params["measurement-noise-covariance"]);
      	pf.setMeasurementNoiseCovariance(mnc);
      } else
      	throw factory_error("Measurement noise covariance missing.");

      if (params.count("initial-state")) {
      	VectorXd is = boost::any_cast<VectorXd>(params["initial-state"]);
      	pf.setInitialState(is);
      } else if (params.count("state-bounds")) {
      	MatrixXd is = boost::any_cast<MatrixXd>(params["state-bounds"]);

      	// do some validation here: each row of 'is' represents a
      	// bound-tuple (a,b) -> #col = 2, #rows = state-size
      	if (params.count("state-size")) {
      	  int n = boost::any_cast<int>(params["state-size"]);
      	  if (is.rows() != n)
      	    throw factory_error("Invalid number of state bounds.");
      	}
      	if (is.cols() != 2)
      	  throw factory_error("Invalid state bounds.");

      	VectorXd a = is.col(0);
      	VectorXd b = is.col(1);
      	pf.initializeParticles(a, b);
      } else
      	throw factory_error("Initial state / state bounds missing.");

      // optional -----    

      if (params.count("control-input-size")) {
      	int cis = boost::any_cast<int>(params["control-input-size"]);
      	pf.setControlInputSize(cis);
      }

      pf.validate();	// throws on error
    } catch(std::exception& e) {
      std::string additionalInfo = "Initializing ParticleFilterSIR failed. ";
      throw factory_error(additionalInfo + e.what());
    }
  }
  
  void EstimatorFactory::initConfidenceWeightedAveraging(estimation::ConfidenceWeightedAveraging& cwa)
  {
    // required: none
    // optional: ignore-zero-variance-values

    try {
      if (params.count("ignore-zero-variance-values")) {
	cwa.setIgnoreZeroVarianceValues();	// param available so set flag
      }
    } catch(std::exception& e) {
      std::string additionalInfo = "Initializing ConfidenceWeightedAveraging failed. ";
      throw factory_error(additionalInfo + e.what());
    }
  }
}
