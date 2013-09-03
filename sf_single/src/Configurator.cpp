/**
 * @file 
 * @author Denise Ratasich
 * @date 03.09.2013
 *
 * @brief Implementation of the configurator.
 */

#include "Configurator.h"

estimation::IEstimationMethod* Configurator::getInitializedEstimator (void)
{
#ifdef METHOD
  std::string method = METHOD;
#else
  throw std::runtime_error("Method not specified!");
#endif

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

  throw std::runtime_error("Unknown method! "
			   "See help for available methods.");
}

// -----------------------------------------
// initialization of the specific estimator
// -----------------------------------------

void Configurator::initMovingMedian(estimation::MovingMedian& mm)
{
  // required: none
  // optional: WINDOW_SIZE

#ifdef WINDOW_SIZE
  mm.setWindowSize(WINDOW_SIZE);
#endif
}

void Configurator::initMovingAverage(estimation::MovingAverage& ma)
{
  // required: none
  // optional: WINDOW_SIZE, WEIGHTING_COEFFICIENTS

  unsigned int ws = ma.getWindowSize();	// default window size
#ifdef WINDOW_SIZE
  ws = WINDOW_SIZE;
  ma.setWindowSize(ws);
#endif

#ifdef WEIGHTING_COEFFICIENTS
  double wc[] = { WEIGHTING_COEFFICIENTS };
  int wcSize = sizeof(wc)/sizeof(double);
  
  if (wcSize == ws) {
    // only b_k coefficients are passed
    ma.setWeightingCoefficientsIn(&wc[0], ws);
  } else if (wcSize == 2*ws-1) {
    // a_k coefficients also passed through arguments
    // first (ws-1) numbers
    ma.setWeightingCoefficientsOut(&wc[0], ws-1);
    ma.setWeightingCoefficientsIn(&wc[ws-1], ws);
  } else {
    throw std::length_error("Invalid number of"
		    "weighting-coefficients "
		    "(must equal "
		    "1x or 2x-1 window size).");
  }
#endif
}

void Configurator::initKalmanFilter(estimation::KalmanFilter& kf)
{
  // required: state-transition-model, process-noise-covariance,
  //   observation-model, measurement-noise-covariance
  // optional: control-input-model, control-input, initial-state,
  //   initial-error-covariance

  // required -----

#ifdef STATE_TRANSITION_MODEL
  std::vector< std::vector<double> > stm = { STATE_TRANSITION_MODEL };
  kf.setStateTransitionModel(stm);
#else
  throw std::runtime_error("State transition model missing.");
#endif

#ifdef PROCESS_NOISE_COVARIANCE
  std::vector< std::vector<double> > pnc = { PROCESS_NOISE_COVARIANCE };
  kf.setProcessNoiseCovariance(pnc);
#else
  throw std::runtime_error("Process noise covariance missing.");
#endif

#ifdef OBSERVATION_MODEL
  std::vector< std::vector<double> > om = { OBSERVATION_MODEL };
  kf.setObservationModel(om);
#else
  throw std::runtime_error("Observation model missing.");
#endif

#ifdef MEASUREMENT_NOISE_COVARIANCE
  std::vector< std::vector<double> > mnc = { MEASUREMENT_NOISE_COVARIANCE };
  kf.setMeasurementNoiseCovariance(mnc);
#else
  throw std::runtime_error("Measurement noise covariance missing.");
#endif

  // optional -----

#ifdef CONTROL_INPUT_MODEL
  std::vector< std::vector<double> > cim = { CONTROL_INPUT_MODEL };
  kf.setControlInputModel(cim);
#endif

#ifdef CONTROL_INPUT
  std::vector<double> ci = { CONTROL_INPUT };
  kf.setControlInput(ci);
#endif

#ifdef INITIAL_STATE
  std::vector<double> is = { INITIAL_STATE };
  kf.setInitialState(is);
#endif

#ifdef INITIAL_ERROR_COVARIANCE
  std::vector< std::vector<double> > iec = { INITIAL_ERROR_COVARIANCE };
  kf.setInitialErrorCovariance(iec);
#endif

  kf.validate();	// throws on error
}
