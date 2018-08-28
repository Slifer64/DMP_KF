#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

  GatingFunction::GatingFunction()
  {}

  void GatingFunction::init(double u0, double u_end)
  {
    this->setGatingFunParams(u0, u_end);
  }

  void GatingFunction::setGatingFunParams(double u0, double u_end)
  {
    this->u0 = u0;
    this->a_u = this->u0 - u_end;
  }

  //double GatingFunction::getOutput(double x) const = 0;
  //arma::rowvec GatingFunction::getOutput(const arma::rowvec &x) const = 0;

  //double GatingFunction::getOutputDot(double x) const = 0;
  //arma::rowvec GatingFunction::getOutputDot(const arma::rowvec &x) const = 0;


} // namespace as64_
