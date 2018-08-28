/** Canonical Clock class (abstract)
 * Implements a canonical clock, x = f(t), t:[0 tau] -> x:[x0 x_end].
 * x0 = 0.0 , x_end = 1.0
 * x is the phase variable (clock's output) and tau is a scaling factor
 *  defining the total time duration. The phase variable can exceed
 *  x_end, which in turn means the time t exceeded tau.
 */

#ifndef CANONICAL_CLOCK_H
#define CANONICAL_CLOCK_H

#include <armadillo>

namespace as64_
{

class CanonicalClock
{
public:
//methods (public)
  /** \brief Canonical Clock Constructor.
   */
  CanonicalClock();

  /** \brief Initializes the canonical clock.
   *  @param[in] tau Total time duration (used to set/scale the clocks time duration).
   */
  virtual void init(double tau) = 0;

  /** \brief Sets/scales the canonical clock's time duration.
   *  @param[in] tau Total time duration (used to set/scale the clocks time duration).
   */
  void setTau(double tau);

  /** \brief Returns the canonical clock's time duration.
   *  @return Total time duration (used to set/scale the clocks time duration).
   */
  double getTau() const;

  /** \brief Returns the phase variable derivative of the canonical clock for
   *  the specified phase variable value.
   *  @param[in] x The phase variable value.
   *  @return The phase variable derivative.
   */
  virtual double getPhaseDot(double x) const = 0;

  /** \brief Returns the phase variable derivative of the canonical clock for
   *  the specified phase variable values.
   *  @param[in] x Vector of the phase variable values.
   *  @return Vector of phase variable derivatives.
   */
  virtual arma::rowvec getPhaseDot(const arma::rowvec &x) const = 0;

  /** \brief Returns the output of the canonical clock for the specified timestamps.
   *  @param[in] t Timestamp.
   *  @return The phase variable value for timestamp 't'.
   */
  virtual double getPhase(double t) const = 0;

  /** \brief Returns the output of the canonical clock for the specified timestamps.
   *  @param[in] t Vector of timestamps.
   *  @return Vector with the phase variable values at each timestamp in 't'.
   */
  virtual arma::rowvec getPhase(const arma::rowvec &t) const = 0;

protected:
//properties (private)
  double x0; ///< initial value of the phase variable
  double x_end; ///< final value of the phase variable
  double tau; ///< total time duration

//methods (private)
  void init_helper(double tau);

}; // class CanonicalClock

} // namespace as64_

#endif // CANONICAL_CLOCK_H
