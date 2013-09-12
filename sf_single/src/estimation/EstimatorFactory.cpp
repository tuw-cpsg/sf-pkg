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
	vector wc = boost::any_cast<vector>(params["weighting-coefficients"]);
  
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
	matrix stm = boost::any_cast<matrix>(params["state-transition-model"]);
	kf.setStateTransitionModel(stm);
      } else
	throw factory_error("State transition model missing.");

      if (params.count("process-noise-covariance")) {
	matrix pnc = boost::any_cast<matrix>(params["process-noise-covariance"]);
	kf.setProcessNoiseCovariance(pnc);
      } else
	throw factory_error("Process noise covariance missing.");

      if (params.count("observation-model")) {
	matrix om = boost::any_cast<matrix>(params["observation-model"]);
	kf.setObservationModel(om);
      } else
	throw factory_error("Observation model missing.");

      if (params.count("measurement-noise-covariance")) {
	matrix mnc =  boost::any_cast<matrix>(params["measurement-noise-covariance"]);
	kf.setMeasurementNoiseCovariance(mnc);
      } else
	throw factory_error("Measurement noise covariance missing.");

      // optional -----

      if (params.count("control-input-model")) {
	matrix cim = boost::any_cast<matrix>(params["control-input-model"]);
	kf.setControlInputModel(cim);
      }

      if (params.count("control-input")) {
	vector ci = boost::any_cast<vector>(params["control-input"]);
	kf.setControlInput(ci);
      }

      if (params.count("initial-state")) {
	vector is = boost::any_cast<vector>(params["initial-state"]);
	kf.setInitialState(is);
      }

      if (params.count("initial-error-covariance")) {
	matrix iec = boost::any_cast<matrix>(params["initial-error-covariance"]);
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
    //   observation-model, observation-model-covariance,
    //   measurement-noise-covariance

    // optional: control-input, initial-state,
    //   initial-error-covariance

    // required -----
    try {
      if (params.count("state-transition-model")) {
	ExtendedKalmanFilter::func_f stm = boost::any_cast<ExtendedKalmanFilter::func_f>(params["state-transition-model"]);
	ekf.setStateTransitionModel(stm);
      } else
	throw factory_error("State transition model missing.");

      if (params.count("state-transition-model-jacobian")) {
	ExtendedKalmanFilter::func_df stmj = boost::any_cast<ExtendedKalmanFilter::func_df>(params["state-transition-model-jacobian"]);
	ekf.setJacobianOfStateTransitionModel(stmj);
      } else
	throw factory_error("Jacobian of state transition model missing.");

      if (params.count("process-noise-covariance")) {
	matrix pnc = boost::any_cast<matrix>(params["process-noise-covariance"]);
	ekf.setProcessNoiseCovariance(pnc);
      } else
	throw factory_error("Process noise covariance missing.");

      if (params.count("observation-model")) {
	ExtendedKalmanFilter::func_h om = boost::any_cast<ExtendedKalmanFilter::func_h>(params["observation-model"]);
	ekf.setObservationModel(om);
      } else
	throw factory_error("Observation model missing.");

      if (params.count("observation-model-jacobian")) {
	ExtendedKalmanFilter::func_dh omj = boost::any_cast<ExtendedKalmanFilter::func_dh>(params["observation-model-jacobian"]);
	ekf.setJacobianOfObservationModel(omj);
      } else
	throw factory_error("Jacobian of observation model missing.");

      if (params.count("measurement-noise-covariance")) {
	matrix mnc =  boost::any_cast<matrix>(params["measurement-noise-covariance"]);
	ekf.setMeasurementNoiseCovariance(mnc);
      } else
	throw factory_error("Measurement noise covariance missing.");

      // optional -----

      if (params.count("control-input")) {
	vector ci = boost::any_cast<vector>(params["control-input"]);
	ekf.setControlInput(ci);
      }

      if (params.count("state-size")) {
	int n = boost::any_cast<int>(params["state-size"]);
	ekf.setSizeOfState(n);
      }

      if (params.count("initial-state")) {
	vector is = boost::any_cast<vector>(params["initial-state"]);
	ekf.setInitialState(is);
      }

      if (params.count("initial-error-covariance")) {
	matrix iec = boost::any_cast<matrix>(params["initial-error-covariance"]);
	ekf.setInitialErrorCovariance(iec);
      }

      ekf.validate();	// throws on error
    } catch(std::exception& e) {
      std::string additionalInfo = "Initializing ExtendedKalmanFilter failed. ";
      throw factory_error(additionalInfo + e.what());
    }
  }
}
