/**
 * @file 
 * @author Denise Ratasich
 * @date 05.09.2013
 *
 * @brief Collects things for configuration needed by the ROS node.
 *
 * Collects macros needed by the user for config.h and macros needed
 * for code generation and initialization of the Configurator.
 */

#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#include "estimation/EstimatorFactory.h"

// -----------------------------------------
// file members
// -----------------------------------------
/**
 * @brief Initializes the estimator factory.
 */
void initEstimatorFactory(estimation::EstimatorFactory& factory);

// -----------------------------------------
// includes for the ROS node
// -----------------------------------------
/** @brief Bring in the current configuration. */
#include "config.h"

// includes according configuration
#include TOPIC_INCLUDE

#endif
