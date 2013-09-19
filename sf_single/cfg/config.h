/**
 * @file 
 * @author Denise Ratasich
 * @date 19.09.2013
 *
 * @brief Configuration header.
 *
 * Contains the defines which are used to create a special estimator.
 * According to the estimation method, chosen with the macro METHOD,
 * specific additional parameters have to be set. Strings have to be
 * set in quotes.
 *
 * Possible parameters for each method can be found in the
 * documentation of each method:
 * - \ref movingaverage
 * - \ref movingmedian
 * - \ref kalmanfilter
 * - \ref extendedkalmanfilter
 * - \ref unscentedkalmanfilter
 *
 * \example moving-median_1.h
 * \example moving-average_1.h
 * \example moving-average_2.h
 * \example kalman-filter_1.h
 * \example kalman-filter_2.h
 * \example extended-kalman-filter_1.h
 * \example unscented-kalman-filter_1.h
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

// Place your configuration here. Include or insert it directly, I
// chose to include an example.
#include "../examples/unscented-kalman-filter_1.h"

#endif
