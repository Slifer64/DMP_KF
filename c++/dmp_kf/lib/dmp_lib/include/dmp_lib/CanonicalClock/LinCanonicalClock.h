/** Linear Canonical Clock class
 * Implements a linear canonical clock, x = f(t), t:[0 tau] -> x:[x0 x_end].
 * The clock's evolution is defined as:
 *    tau*dx = a_x
 * where x is the phase variable (clock's output), a_x a constant and
 * tau is a scaling factor defining the total time duration. The phase
 * variable can exceed x_end, which in turn means the time t exceeded tau.
 */

#ifndef LINEAR_CANONICAL_CLOCK_H
#define LINEAR_CANONICAL_CLOCK_H

#include <dmp_lib/CanonicalClock/CanonicalClock.h>

namespace as64_
{

class LinCanonicalClock: public CanonicalClock
{
//methods (public)
public:
  /** \brief Linear Canonical Clock Constructor.
   *  @param[in] tau Total time duration (used to set/scale the clocks time duration).
   */
  LinCanonicalClock(double tau = 1.0);

  /** \brief Initializes the canonical clock.
   *  @param[in] tau Total time duration (used to set/scale the clocks time duration).
   */
  virtual void init(double tau);

  /** \brief Returns the phase variable derivative of the canonical clock for
   *  the specified phase variable value.
   *  @param[in] x The phase variable value.
   *  @return The phase variable derivative.
   */
  virtual double getPhaseDot(double x) const;

  /** \brief Returns the phase variable derivative of the canonical clock for
   *  the specified phase variable values.
   *  @param[in] x Vector of the phase variable values.
   *  @return Vector of phase variable derivatives.
   */
  virtual arma::rowvec getPhaseDot(const arma::rowvec &x) const;

  /** \brief Returns the output of the canonical clock for the specified timestamps.
   *  @param[in] t Timestamp.
   *  @return The phase variable value for timestamp 't'.
   */
  virtual double getPhase(double t) const;

  /** \brief Returns the output of the canonical clock for the specified timestamps.
   *  @param[in] t Vector of timestamps.
   *  @return Vector with the phase variable values at each timestamp in 't'.
   */
  virtual arma::rowvec getPhase(const arma::rowvec &t) const;

//properties (private)
private:
  double a_x; ///< the rate of evolution of the phase variable

}; // class LinCanonicalClock

} // namespace as64_

#endif // LINEAR_CANONICAL_CLOCK_H
