/**
 * @file 
 * @author Denise Ratasich
 * @date 28.11.2013
 *
 * @brief Collects the function pointers for models.
 */

#ifndef __ESTIMATION_MODELS_H__
#define __ESTIMATION_MODELS_H__

#include <Eigen/Dense>
using namespace Eigen;

namespace estimation
{
  /** 
   * @brief Pointer to a function representing the state transition
   * model.
   *
   * @param x The state x at time k-1 (input) used to calculate the
   * estimated (a priori) state vector at time k (output).
   * @param The control input at time k-1.
   */
  typedef void (*func_f)(VectorXd& x, const VectorXd& u);

  /** 
   * @brief Pointer to a function calculating the Jacobian of the state
   * transition model.
   */
  typedef void (*func_df)(MatrixXd& A, const VectorXd& x, const VectorXd& u);

  /** 
   * @brief Pointer to a function representing the observation model.
   * 
   * @param z The estimated measurement vector.
   * @param x The a priori state vector.
   */
  typedef void (*func_h)(VectorXd& z, const VectorXd& x);

  /** 
   * @brief Pointer to a function calculating the Jacobian of the
   * observation model.
   */
  typedef void (*func_dh)(MatrixXd& H, const VectorXd& x);
}

#endif
