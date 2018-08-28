/** Constant Gating Function class
 *  Implements a Constant gating function, u=f(x), x:[0 1]->u:[u_const u_const],
 *  where u_const is the constant output value.
 * The output of the gating function is:
 *    u = u_const;
 *   du = 0.0;
 */

#ifndef CONSTANT_GATING_FUNCTION_H
#define CONSTANT_GATING_FUNCTION_H

#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

class ConstGatingFunction: public GatingFunction
{
public:
  /** \brief Constant Gating Function Constructor.
   */
  ConstGatingFunction(double u0 = 1.0, double u_end = 1.0);

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

}; // class ConstGatingFunction

} // namespace as64_

#endif // CONSTANT_GATING_FUNCTION_H
