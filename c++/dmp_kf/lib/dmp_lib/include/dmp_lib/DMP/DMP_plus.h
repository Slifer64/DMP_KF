#ifndef DYNAMICAL_MOVEMENT_PRIMITIVE_PLUS_H
#define DYNAMICAL_MOVEMENT_PRIMITIVE_PLUS_H

#include <dmp_lib/DMP/DMP_.h>

namespace as64_
{

class DMP_plus: public DMP_
{
public:
  DMP_plus();

  DMP_plus(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr=NULL);

  virtual double train(const arma::rowvec &Time, const arma::rowvec &yd_data,
    const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double y0, double g,
    const std::string &train_method = "LWR", bool ret_train_err=false);

  virtual double learnedForcingTerm(double x, double y0, double g) const;

  virtual double forcingTerm(double x) const;

  virtual double shapeAttractor(double x, double y0, double g) const;

  virtual arma::vec kernelFunction(double x) const;

  virtual void parseExtraArgs(const param_::ParamList *paramListPtr);

private:
  arma::vec b; ///< N_kernelsx1 vector with the bias term for each weight
  int k_trunc_kernel; ///< gain multiplied by the std of each kernel to define the truncated kernels width

}; // class DMP_plus

} // namespace as64_

#endif // DYNAMICAL_MOVEMENT_PRIMITIVE_PLUS_H
