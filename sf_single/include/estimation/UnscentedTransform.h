/**
 * @file 
 * @author Denise Ratasich
 * @date 17.09.2013
 *
 * @brief Header file of the unscented Kalman filter.
 */

#ifndef __ESTIMATION_UNSCENTED_TRANSFORM_H__
#define __ESTIMATION_UNSCENTED_TRANSFORM_H__

#include <vector>
#include <Eigen/Dense>
#include "estimation/ITransformer.h"

using namespace Eigen;

namespace estimation 
{
  /**
   * @brief Unscented Transform.
   *
   * Calculates the statistics of a random variable x which is
   * propagated through a nonlinear function y = f(x).
   *
   * This is done by selecting specific samples called sigma points
   * around the mean of x according P, the covariance of x.
   *
   * \f{eqnarray*}{
   *   \chi_0 &=& \hat x \\ 
   *   \chi_i &=& \hat x + ( \sqrt{(L+\lambda)P} )_i     \quad i = 1,...,L \\
   *   \chi_i &=& \hat x - ( \sqrt{(L+\lambda)P} )_{i-n} \quad i = L+1,...,2L
   * \f}
   *
   * - \f$ \sqrt{(L+\lambda)P} \f$ is a matrix square root of size
   *   nxn.
   * - \f$ L \f$ dimension of x.
   * - \f$ \lambda = \alpha^2 (L + \kappa) - n \f$ is a scaling
   *   parameter.
   *   - \f$ \alpha \f$ sets the spread of the sigma points, usually a
   *     value between \f$ 1 \le \alpha \le 10^{-4}\f$.
   *   - \f$ \kappa \f$ usually set to \f$ 3 - L \f$.
   *   - \f$ \beta \f$ integrates prior knowledge of distribution into
   *     the sigma points. When x is Gaussian distributed set \f$\beta
   *     = 2\f$.
   *
   * The approximation is done with weighted averaging, following
   * weights are used (the weights for mean (m) and covariance (c)
   * only differ in the weight for sigma point 0, the original mean):
   *
   * \f{eqnarray*}{
   *   W_0^{(m)} &=& \frac{\lambda}{L + \lambda} \\ 
   *   W_0^{(c)} &=& \frac{\lambda}{L + \lambda} + 1 - \alpha^2 + \beta \\ 
   *   W_i^{(m)} = W_i^{(c)} &=& \frac{1}{2(L + \lambda)} \\
   * \f}
   */ 
  class UnscentedTransform 
  {
  private:
    /** @brief Approximated mean of x propagated through function
     * f. */
    VectorXd y;
    /** @brief Approximated covariance of x propagated through
     * function f. */
    MatrixXd Py;
    /** @brief Approximated cross-covariance Pxy of x and y. Size: L x
     * L. */
    MatrixXd Pxy;

    // -----------------------------------------
    // parameters
    // -----------------------------------------
    /** @brief Mean of x. Size: L. */
    VectorXd x;
    /** @brief Covariance of x. Size: L x L. */
    MatrixXd Px;
    /** @brief Transformer which implements a function to propagate a
     * sigma point through. */
    ITransformer* transformer;

    /** @brief Scaling factor for spread. */
    double alpha;
    /** @brief Scaling factor for distribution type. */
    double beta;
    /** @brief TODO Scaling factor - what for???. */
    double kappa;

  public:
    /**
     * @brief Basic constructor.
     *
     * Throws on invalid sizes. Size of x must equal number of rows
     * and columns of P. Further the returned vector of function f
     * must have the size of x. Because f == 0 makes no sense, this
     * constructor throws a runtime_error when 0 is passed.
     *
     * @param x Mean of the random variable x.
     * @param P Covariance of the random variable x.
     * @param transformer Object containing a suitable function which
     * is able to propagate a sigma point.
     */
    UnscentedTransform (VectorXd x, MatrixXd P, ITransformer* transformer);

    /**
     * @brief Destructor of this class.
     */
    ~UnscentedTransform ();

    /**
     * @brief Does the approximation of mean and covariance.
     * 
     * The sigma points are propagated through the function. The mean
     * and covariance is calculated by the weighted sum of these
     * posterior sigma points.
     *
     * \note Before calling this function the mean and covariance is
     * zero.
     */
    void compute (void);

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /**
     * @brief Returns the approximated mean y.
     *
     * @return The approximated mean.
     */
    VectorXd mean (void) const;

    /** 
     * @brief Returns the approximated covariance Py.
     *
     * @return The approximated covariance.
     */
    MatrixXd covariance (void) const;

    /** 
     * @brief Returns the approximated covariance Pxy.
     *
     * @return The approximated cross-covariance Pxy.
     */
    MatrixXd crossCovarianceXY (void) const;

  private:
    /** 
     * @brief Generates the sigma points.
     *
     * The sigma points are constructed according to \cite Wan01
     * (scaled unscented transform). The parameters for the generation
     * can only be passed via constructor. The calculation uses LL^T
     * (standard) Cholesky decomposition.
     *
     * For the approximation of mean y and covariance Py weights are
     * needed, which are determined in this function. Mean and
     * Covariance use the same weights except for the first sigma
     * point (a priori mean x and covariance Px). But to save memory
     * only one array of weights is used and filled in following
     * order: 
     * - W_0 for mean
     * - W_1, W_2, ..., W_2L weigths for mean and covariance for sigma
     *   points 1 to 2L
     * - W_0 for covariance; its the last element so pull it off this
     *   array \c weights when approximating the mean; then replace
     *   the first element with the pulled weight to approximate the
     *   covariance
     * 
     * @param sigmaPoints The array where the sigma points should be
     * stored to. Must be empty initially.
     * @param weights The associated weights for the sigma
     * points. Must be empty initially.
     */
    void generateSigmaPoints(std::vector<VectorXd>& sigmaPoints, std::vector<double>& weights);
  };

}

#endif
