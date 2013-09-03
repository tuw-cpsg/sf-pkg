/**
 * @file 
 * @author Denise Ratasich
 * @date 03.09.2013
 *
 * @brief Header file of the Configurator.
 */

#ifndef __CONFIGURATOR_H__
#define __CONFIGURATOR_H__

#include "estimation/IEstimationMethod.h"
#include "estimation/methods.h"
#include "config.h"

/**
 * @brief Configurator, initializes and returns an estimator.
 *
 * Provides methods to initialize the estimation method specified in
 * the header-file.
 */
class Configurator
{

public: 
  /**
   * @brief Creates an estimator according to the specified method and
   * does the specific initialization.
   *
   * Throws on error, e.g. unknown method, invalid parameter.
   *
   * @return The estimator.
   */
  static estimation::IEstimationMethod* getInitializedEstimator (void);

private:
  // -----------------------------------------
  // initialization of the specific estimator
  // -----------------------------------------
  static void initMovingMedian(estimation::MovingMedian& mm);
  static void initMovingAverage(estimation::MovingAverage& ma);
  static void initKalmanFilter(estimation::KalmanFilter& kf);
};

#endif
