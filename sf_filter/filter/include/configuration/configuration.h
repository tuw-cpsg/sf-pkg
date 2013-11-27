/**
 * @file 
 * @author Denise Ratasich
 * @date 05.09.2013
 *
 * @brief Collects things for configuration needed by the ROS node.
 *
 * Includes the configuration header from the user. Collects macros
 * needed by the user for config.h and macros needed for code
 * generation and initialization of the Configurator.
 *
 * \example extended-kalman-filter_nav.h
 * \example unscented-kalman-filter_nav.h
 */

#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#include "estimation/EstimatorFactory.h"

// -----------------------------------------
// file members
// -----------------------------------------
/**
 * @brief Returns the estimator's period in ms.
 *
 * This method is called by the ROS node to get the period which
 * should be used for filtering. 
 *
 * @return Filter period in ms.
 */
int getEstimatePeriod(void);

/**
 * @brief Initializes an estimator factory.
 *
 * Passes parameters to an estimator factory, i.e. maps the parameters
 * given in the configuration header to variables for initializing an
 * estimator. Once called, the create() method of a factory can be
 * used to get the initialized estimator.
 *
 * @param factory An estimator factory.
 */
void initEstimatorFactory(estimation::EstimatorFactory& factory);

// -----------------------------------------
// includes for the ROS node
// -----------------------------------------

/** @brief Macros and includes for the configuration. */
#include "configuration/pre_config.h"

/** @brief Bring in the current configuration. */
#include "config.h"

/** @brief Validate the configuration, i.e. checks the parameters
 * given in the configuration header. */
#include "validation.h"

/** @brief Prepares configuration for the ROS node. */
#include "configuration/post_config.h"

#endif
