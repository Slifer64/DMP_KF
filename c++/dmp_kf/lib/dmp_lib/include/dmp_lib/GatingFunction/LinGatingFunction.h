/** Linear Gating Function class
 * Implements a linear gating function, u=f(x), x:[0 1]->u:[u0 u_end],
 * where u0 is the initial and u_end the final value.
 * The output of the gating function is:
 *    u = u0 - a_u*x;
 *   du = -a_u;
 */

#ifndef LINEAR_GATING_FUNCTION_H
#define LINEAR_GATING_FUNCTION_H

#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

class LinGatingFunction: public GatingFunction
{
public:
  /** \brief Linear Gating Function Constructor.
   */
  LinGatingFunction(double u0 = 1.0, double u_end = 0.005);

  /** \brief Initializes the gating function.
   *  @param[in] u0 Initial value of the gating function.
   *  @param[in] u_end Final value of the gating function.
   */
  virtual void init(double u0, double u_end);

  /** \brief Sets the gating function's time constants based on the value of the phase variable at the end of the movement.
   *  @param[in] u0 Initial value of the gating function.
   *  @param[in] u_end Final value of the gating function.
   */
  virtual void setGatingFunParams(double u0, double u_end);

  /** \brief Returns the gating function's output for the specified timestamps.
   *  @param[in] x A timestamp or vector of timestamps.
   *  @return u Value or vector of values of the gating function's output.
   */
  virtual double getOutput(double x) const;
  virtual arma::rowvec getOutput(const arma::rowvec &x) const;

  /** \brief Returns the gating function's derivated output for the specified timestamps.
   *  @param[in] x A timestamp or vector of timestamps.
   *  @return u Value or vector of values of the gating function's derivated output.
   */
  virtual double getOutputDot(double x) const;
  virtual arma::rowvec getOutputDot(const arma::rowvec &x) const;

}; // class LinGatingFunction

} // namespace as64_

#endif // LINEAR_GATING_FUNCTION_H
