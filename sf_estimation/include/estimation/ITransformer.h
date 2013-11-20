/**
 * @file 
 * @author Denise Ratasich
 * @date 18.09.2013
 *
 * @brief Interface for a transformer, y = f(x).
 */

#ifndef __ESTIMATION_ITRANSFORMER_H__
#define __ESTIMATION_ITRANSFORMER_H__

#include <Eigen/Core>

namespace estimation 
{
  /** 
   * @brief Interface for transformers.
   */
  class ITransformer {

  public:
    /** @brief Destructor. */
    virtual ~ITransformer () { };

    /** 
     * @brief A function of type y = f(x).
     *
     * @param x The vector to propagate through this function.
     * @return The result when propagating x through this function.
     */
    virtual Eigen::VectorXd transform (const Eigen::VectorXd& x) = 0;
  };
}

#endif
