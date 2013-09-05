/**
 * @file 
 * @author Denise Ratasich
 * @date 05.09.2013
 *
 * @brief A macro for each method.
 */

#ifndef __CONFIGURATION_METHODS_H__
#define __CONFIGURATION_METHODS_H__

/** 
 * @defgroup METHOD_GROUP Macros for defining the method in \c
 * config.h.
 *
 * Macros can be compared during compile time, strings only at
 * runtime, which is too late e.g. for configuring the EKF which needs
 * code generation. So instead of declaring a method in the
 * configuration file with a string (e.g. #define METHOD
 * "KalmanFilter") it has to be done with these macros.
 *
 * @{
 */
#define MOVING_MEDIAN			1
#define MOVING_AVERAGE			2
#define KALMAN_FILTER			3
#define EXTENDED_KALMAN_FILTER		4
/** @} */

#endif
