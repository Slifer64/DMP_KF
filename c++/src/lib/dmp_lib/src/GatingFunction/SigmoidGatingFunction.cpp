#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

namespace as64_
{

  SigmoidGatingFunction::SigmoidGatingFunction(double u0, double u_end)
  {
    this->init(u0, u_end);
  }

  void SigmoidGatingFunction::init(double u0, double u_end)
  {
    this->a_u = 700.0;
    this->u0 = u0;
    this->c = 1.0 - (1.0/this->a_u)*std::log((this->u0-u_end)/u_end);
  }

  double SigmoidGatingFunction::getOutput(double x) const
  {
    double exp_t = std::exp((this->a_u)*(x-this->c));
    double u = this->u0 * 1.0 / (1.0 + exp_t);
    return u;
  }

  arma::rowvec SigmoidGatingFunction::getOutput(const arma::rowvec &x) const
  {
    arma::rowvec exp_t = arma::exp((this->a_u)*(x-this->c));
    arma::rowvec u = this->u0 * 1.0 / (1.0 + exp_t);
    return u;
  }

  double SigmoidGatingFunction::getOutputDot(double x) const
  {
    double exp_t = std::exp((this->a_u)*(x-this->c));
    double du = -this->u0 * (this->a_u) * exp_t / std::pow(1.0 + exp_t, 2);
    return du;
  }

  arma::rowvec SigmoidGatingFunction::getOutputDot(const arma::rowvec &x) const
  {
    arma::rowvec exp_t = arma::exp((this->a_u)*(x-this->c));
    arma::rowvec du = -this->u0 * (this->a_u) * exp_t / arma::pow(1.0 + exp_t, 2);
    return du;
  }

  double SigmoidGatingFunction::getPartDev_1oTau(double t, double x) const
  {
    double h = this->getOutput(x);
    return -this->a_u * t * h * (1-h);
  }

} // namespace as64_
