#include <dmp_lib/GatingFunction/ExpGatingFunction.h>

namespace as64_
{

  ExpGatingFunction::ExpGatingFunction(double u0, double u_end)
  {
    this->init(u0, u_end);
  }


  void ExpGatingFunction::init(double u0, double u_end)
  {
    this->u0 = u0;
    this->a_u = this->u0 - u_end;

    this->u0 = u0;
    double x = u_end/this->u0;

    if (u0 == 0)
    {
      throw std::runtime_error("ExpGatingFunction: setGatingFunParams: u0 must be != 0\n");
    }

    if (x <= 0)
    {
        throw std::runtime_error("ExpGatingFunction: setGatingFunParams: u0 and u_end must be both positive or negative.\n");
    }

    this->a_u = -std::log(x);
  }


  double ExpGatingFunction::getOutput(double x) const
  {
    double u = this->u0*std::exp(-this->a_u*x);
    return u;
  }


  arma::rowvec ExpGatingFunction::getOutput(const arma::rowvec &x) const
  {
    arma::rowvec u = this->u0*arma::exp(-this->a_u*x);
    return u;
  }


  double ExpGatingFunction::getOutputDot(double x) const
  {
    double du = -this->a_u*this->u0*std::exp(-this->a_u*x);
    return du;
  }


  arma::rowvec ExpGatingFunction::getOutputDot(const arma::rowvec &x) const
  {
    arma::rowvec du = -this->a_u*this->u0*arma::exp(-this->a_u*x);
    return du;
  }

  double ExpGatingFunction::getPartDev_1oTau(double t, double x) const
  {
    return -this->a_u * t * this->getOutput(x);
  }

} // namespace as64_
