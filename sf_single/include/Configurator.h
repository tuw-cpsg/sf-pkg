/**
 * @file 
 * @author Denise Ratasich
 * @date 03.09.2013
 *
 * @brief Header file of the Configurator.
 */

#ifndef __CONFIGURATOR_H__
#define __CONFIGURATOR_H__

#include <map>
#include <stdexcept>
//#include <ostream>
#include <boost/any.hpp>
#include "estimation/IEstimationMethod.h"
#include "estimation/methods.h"

/**
 * @brief Exception thrown by a Configurator.
 */
class ConfiguratorException : public std::runtime_error
{
public:
  ConfiguratorException(const std::string& info) 
    : std::runtime_error(info) { }
};
typedef ConfiguratorException config_error;

/**
 * @brief Initializes and returns an estimator.
 *
 * Provides methods to initialize the estimation method specified by a
 * parameter list.
 */
class Configurator
{
  std::map<std::string, boost::any> params;

public: 

  typedef std::vector<double> vector;
  typedef std::vector< std::vector<double> > matrix;

  /**
   * @brief Adds a parameter to this configurator.
   *
   * @param key The (unique) name of the parameter.
   * @param value The value of the parameter (e.g. a value, a vector,
   * a matrix, ..).
   */
  void addParam(std::string key, boost::any value);

  /**
   * @brief Deletes all parameters of this configurator.
   */
  void reset(void);
  
  /**
   * @brief Creates an estimator according to the specified method and
   * does the specific initialization.
   *
   * Throws on error, e.g. unknown method, invalid parameter.
   *
   * @return The estimator.
   */
  estimation::IEstimationMethod* getInitializedEstimator (void);

  /**
   * Overloads << operator to be able to print information of this
   * class.
   */
  std::ostream& operator<<(std::ostream& lhs) const;

private:
  // -----------------------------------------
  // initialization of the specific estimator
  // -----------------------------------------
  void initMovingMedian(estimation::MovingMedian& mm);
  void initMovingAverage(estimation::MovingAverage& ma);
  void initKalmanFilter(estimation::KalmanFilter& kf);
};

#endif
