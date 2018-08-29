/** Canonical Clock class (abstract)
 * Implements a canonical clock, x = f(t), t:[0 tau] -> x:[x0 x_end].
 * x0 = 0.0 , x_end = 1.0
 * x is the phase variable (clock's output) and tau is a scaling factor
 *  defining the total time duration. The phase variable can exceed
 *  x_end, which in turn means the time t exceeded tau.
 */

 /** Canonical Clock class
  * Implements a linear canonical clock, x = f(t), t:[0 tau] -> x:[x0 x_end].
  * The clock's evolution is defined as:
  *    tau*dx = a_x
  * where x is the phase variable (clock's output), a_x a constant and
  * tau is a scaling factor defining the total time duration. The phase
  * variable can exceed x_end, which in turn means the time t exceeded tau.
  */

#ifndef DMP_CANONICAL_CLOCK_H
#define DMP_CANONICAL_CLOCK_H

#include <armadillo>

namespace as64_
{

class CanonicalClock
{
public:
//methods (public)
  /** \brief Canonical Clock Constructor.
   *  @param[in] tau Time scaling factor (optinal, default = 1.0).
   */
  CanonicalClock(double tau = 1.0);

  /** \brief Initializes the canonical clock.
   *  @param[in] tau Time scaling factor.
   */
  void init(double tau);

  /** \brief Sets the time scaling factor.
   *  @param[in] tau Time scaling factor.
   */
  void setTau(double tau);

  /** \brief Returns the canonical clock's time scaling.
   *  @return Time scaling factor.
   */
  double getTau() const;

  /** \brief Returns the phase variable derivative of the canonical clock for
   *  the specified phase variable value.
   *  @param[in] x The phase variable value.
   *  @return The phase variable derivative.
   */
  double getPhaseDot(double x) const;

  /** \brief Returns the phase variable derivative of the canonical clock for
   *  the specified phase variable values.
   *  @param[in] x Vector of the phase variable values.
   *  @return Vector of phase variable derivatives.
   */
  arma::rowvec getPhaseDot(const arma::rowvec &x) const;

  /** \brief Returns the output of the canonical clock for the specified timestamps.
   *  @param[in] t Timestamp.
   *  @return The phase variable value for timestamp 't'.
   */
  double getPhase(double t) const;

  /** \brief Returns the output of the canonical clock for the specified timestamps.
   *  @param[in] t Vector of timestamps.
   *  @return Vector with the phase variable values at each timestamp in 't'.
   */
  arma::rowvec getPhase(const arma::rowvec &t) const;

protected:
//properties (private)
  double x0; ///< initial value of the phase variable
  double x_end; ///< final value of the phase variable
  double a_x; ///< the rate of evolution of the phase variable
  double tau; ///< time scaling

}; // class CanonicalClock

} // namespace as64_

#endif // DMP_CANONICAL_CLOCK_H
