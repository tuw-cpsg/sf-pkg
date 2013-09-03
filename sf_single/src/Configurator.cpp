/**
 * @file 
 * @author Denise Ratasich
 * @date 03.09.2013
 *
 * @brief Implementation of the configurator.
 */

#include "Configurator.h"

void Configurator::addParam(std::string key, boost::any value)
{
  params[key] = value;
}

void Configurator::reset(void)
{
  params.clear();
}

estimation::IEstimationMethod* Configurator::getInitializedEstimator (void)
{
  if (params.count("method") == 0)
    throw config_error("Method not specified.");

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
  } catch(std::exception& e) {
    throw config_error(std::string("Configuration failed. ") + e.what());
  }

  throw config_error("Configuration failed. Unknown method.");
}

/*
std::ostream& Configurator::operator<<(std::ostream& lhs) const
{
  std::stringstream ss;
  for(std::map<std::string, boost::any>::const_iterator it = (*this).params.begin();
      it != (*this).params.end();
      it++) {
    std::string key = it->first;
    ss << key << std::endl;
  }

  return lhs << ss.str();
}
*/

// -----------------------------------------
// initialization of the specific estimator
// -----------------------------------------

void Configurator::initMovingMedian(estimation::MovingMedian& mm)
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
    throw config_error(additionalInfo + e.what());
  }
}

void Configurator::initMovingAverage(estimation::MovingAverage& ma)
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
    throw config_error(additionalInfo + e.what());
  }
}

void Configurator::initKalmanFilter(estimation::KalmanFilter& kf)
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
      throw config_error("State transition model missing.");

    if (params.count("process-noise-covariance")) {
      matrix pnc = boost::any_cast<matrix>(params["process-noise-covariance"]);
      kf.setProcessNoiseCovariance(pnc);
    } else
      throw config_error("Process noise covariance missing.");

    if (params.count("observation-model")) {
      matrix om = boost::any_cast<matrix>(params["observation-model"]);
      kf.setObservationModel(om);
    } else
      throw config_error("Observation model missing.");

    if (params.count("measurement-noise-covariance")) {
      matrix mnc =  boost::any_cast<matrix>(params["measurement-noise-covariance"]);
      kf.setMeasurementNoiseCovariance(mnc);
    } else
      throw config_error("Measurement noise covariance missing.");

    // optional -----

    if (params.count("control-input-model")) {
      matrix cim = boost::any_cast<matrix>(params["control-input-model"]);
      kf.setControlInputModel(cim);
    }

    if (params.count("control-input")) {
      std::vector<double> ci = boost::any_cast<vector>(params["control-input"]);
      kf.setControlInput(ci);
    }

    if (params.count("initial-state")) {
      std::vector<double> is = boost::any_cast<vector>(params["initial-state"]);
      kf.setInitialState(is);
    }

    if (params.count("initial-error-covariance")) {
      matrix iec = boost::any_cast<matrix>(params["initial-error-covariance"]);
      kf.setInitialErrorCovariance(iec);
    }

    kf.validate();	// throws on error
  } catch(std::exception& e) {
    std::string additionalInfo = "Initializing KalmanFilter failed. ";
    throw config_error(additionalInfo + e.what());
  }
}
