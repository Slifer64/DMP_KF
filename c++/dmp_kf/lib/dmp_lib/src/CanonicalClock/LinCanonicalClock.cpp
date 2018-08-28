#include <dmp_lib/CanonicalClock/LinCanonicalClock.h>

namespace as64_
{

  LinCanonicalClock::LinCanonicalClock(double tau)
  {
    this->init(tau);
  }

  void LinCanonicalClock::init(double tau)
  {
    this->init_helper(tau);
    this->a_x = this->x0 - this->x_end;
  }

  double LinCanonicalClock::getPhaseDot(double x) const
  {
    double dx = -this->a_x/this->getTau();
    return dx;
  }

  arma::rowvec LinCanonicalClock::getPhaseDot(const arma::rowvec &x) const
  {
    arma::rowvec dx = -this->a_x*arma::rowvec().ones(x.size())/this->getTau();
    return dx;
  }

  double LinCanonicalClock::getPhase(double t) const
  {
    double x = this->x0 - this->a_x*t/this->getTau();
    return x;
  }

  arma::rowvec LinCanonicalClock::getPhase(const arma::rowvec &t) const
  {
    arma::rowvec x = this->x0 - this->a_x*t/this->getTau();
    return x;
  }

} // namespace as64_
