#include <dmp_lib/CanonicalClock/CanonicalClock.h>

namespace as64_
{

  CanonicalClock::CanonicalClock(double tau)
  {
    this->init(tau);
  }

  void CanonicalClock::init(double tau)
  {
    this->x0 = 0.0;
    this->x_end = 1.0;
    this->setTau(tau);
    this->a_x = this->x0 - this->x_end;
  }

  double CanonicalClock::getPhaseDot(double x) const
  {
    double dx = -this->a_x/this->getTau();
    return dx;
  }

  arma::rowvec CanonicalClock::getPhaseDot(const arma::rowvec &x) const
  {
    arma::rowvec dx = -this->a_x*arma::rowvec().ones(x.size())/this->getTau();
    return dx;
  }

  double CanonicalClock::getPhase(double t) const
  {
    double x = this->x0 - this->a_x*t/this->getTau();
    return x;
  }

  arma::rowvec CanonicalClock::getPhase(const arma::rowvec &t) const
  {
    arma::rowvec x = this->x0 - this->a_x*t/this->getTau();
    return x;
  }

  void CanonicalClock::setTau(double tau)
  {
    this->tau = tau;
  }

  double CanonicalClock::getTau() const
  {
    return this->tau;
  }

} // namespace as64_
