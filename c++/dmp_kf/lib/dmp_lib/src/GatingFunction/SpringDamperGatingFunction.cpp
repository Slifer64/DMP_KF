#include <dmp_lib/GatingFunction/SpringDamperGatingFunction.h>

namespace as64_
{

  SpringDamperGatingFunction::SpringDamperGatingFunction(double u0, double u_end)
  {
    this->init(u0, u_end);
  }

  void SpringDamperGatingFunction::init(double u0, double u_end)
  {
    this->setGatingFunParams(u0, u_end);

    if (u0*u_end <= 0)
    {
        throw std::runtime_error("SpringDamperGatingFunction: init: u_end and u0 must have the same sign\n");
    }

    if (std::abs(u_end) > std::abs(u0))
    {
        throw std::runtime_error("SpringDamperGatingFunction: init: |u_end| must be less than |u0|\n");
    }

    this->setGatingFunParams(u0, u_end);
  }

  void SpringDamperGatingFunction::setGatingFunParams(double u0, double u_end)
  {
    if (u_end == 0)
    {
        throw std::runtime_error("SpringDamperGatingFunction: setGatingFunParams: u_end must be != 0\n");
    }

    this->u0 = u0;

    this->a_u = -std::log(u_end/this->u0);
  }

  double SpringDamperGatingFunction::getOutput(double x) const
  {
    double exp_at = std::exp(-this->a_u * x);
    double s = 1 / (std::exp(-1)/this->a_u);
    double u = this->u0*s*(x*exp_at );
    return u;
  }

  arma::rowvec SpringDamperGatingFunction::getOutput(const arma::rowvec &x) const
  {
    arma::rowvec exp_at = arma::exp(-this->a_u * x);
    arma::rowvec s = 1 / (arma::exp(-arma::rowvec().ones(x.size()))/this->a_u);
    arma::rowvec u = this->u0*s%(x%exp_at );
    return u;
  }

  double SpringDamperGatingFunction::getOutputDot(double x) const
  {
    double exp_at = std::exp(-this->a_u * x);
    double du = this->u0*exp_at * (1 - this->a_u*x );
    return du;
  }

  arma::rowvec SpringDamperGatingFunction::getOutputDot(const arma::rowvec &x) const
  {
    arma::rowvec exp_at = arma::exp(-this->a_u * x);
    arma::rowvec du = this->u0*exp_at % (1 - this->a_u*x );
    return du;
  }

} // namespace as64_
