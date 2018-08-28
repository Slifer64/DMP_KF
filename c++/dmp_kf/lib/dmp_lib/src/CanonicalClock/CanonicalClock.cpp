#include <dmp_lib/CanonicalClock/CanonicalClock.h>

namespace as64_
{

  CanonicalClock::CanonicalClock()
  {}

  void CanonicalClock::setTau(double tau)
  {
    this->tau = tau;
  }

  double CanonicalClock::getTau() const
  {
    return this->tau;
  }

  void CanonicalClock::init_helper(double tau)
  {
    this->x0 = 0.0;
    this->x_end = 1.0;
    this->setTau(tau);
  }

} // namespace as64_
