#include <dmp_lib/GatingFunction/ConstGatingFunction.h>

namespace as64_
{

  ConstGatingFunction::ConstGatingFunction(double u0, double u_end)
  {
    this->init(u0, u0);
  }

  void ConstGatingFunction::init(double u0, double u_end)
  {
    this->setGatingFunParams(u0, u0);
  }

  void ConstGatingFunction::setGatingFunParams(double u0, double u_end)
  {
    this->u0 = u0;
    this->a_u = 0.0;
  }

  double ConstGatingFunction::getOutput(double x) const
  {
    double u = this->u0;
    return u;
  }

  arma::rowvec ConstGatingFunction::getOutput(const arma::rowvec &x) const
  {
    arma::rowvec u = this->u0*arma::rowvec().ones(size(x));
    return u;
  }

  double ConstGatingFunction::getOutputDot(double x) const
  {
    double du = -this->a_u;
    return du;
  }

  arma::rowvec ConstGatingFunction::getOutputDot(const arma::rowvec &x) const
  {
    arma::rowvec du = -this->a_u * arma::rowvec().ones(x.size());
    return du;
  }

} // namespace as64_
