/**
 * @file 
 * @author Denise Ratasich
 * @date 05.09.2013
 *
 * @brief Macros and includes for the configuration header.
 *
 * Defines a macro for each method. Macros can be compared during
 * compile time, strings only at runtime, which is too late e.g. for
 * configuring the EKF which needs code generation. So instead of
 * declaring a method in the configuration file with a string
 * (e.g. #define METHOD "KalmanFilter") it has to be done with these
 * macros (e.g. #define METHOD KALMAN_FILTER).
 */

#ifndef __CONFIGURATION_PRECONFIG_H__
#define __CONFIGURATION_PRECONFIG_H__

#define MOVING_MEDIAN			1
#define MOVING_AVERAGE			2
#define KALMAN_FILTER			3
#define EXTENDED_KALMAN_FILTER		4

#endif
