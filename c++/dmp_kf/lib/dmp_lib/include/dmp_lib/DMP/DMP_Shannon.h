#ifndef DYNAMICAL_MOVEMENT_PRIMITIVE_SHANNON_H
#define DYNAMICAL_MOVEMENT_PRIMITIVE_SHANNON_H

#include <dmp_lib/DMP/DMP_.h>

namespace as64_
{

class DMP_Shannon: public DMP_
{
public:
  DMP_Shannon();

  DMP_Shannon(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr=NULL);

  virtual double train(const arma::rowvec &Time, const arma::rowvec &yd_data,
    const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double y0, double g,
    const std::string &train_method="", bool ret_train_err=false);

  virtual double calcFd(double x, double y, double dy, double ddy, double y0, double g) const;

  virtual double forcingTerm(double x) const;

  virtual arma::vec kernelFunction(double x) const;

  virtual double learnedForcingTerm(double x, double y0, double g) const;

  virtual void parseExtraArgs(const param_::ParamList *paramListPtr);

private:
  double Freq_min; ///< minimum allowable filter frequency to avoid instabilities with filtering
  double Freq_max; ///< filter out all frequencies beyond 'Freq_max'
  double Wmin; ///< minimum energy percent that must be retained after filtering
  double P1_min; ///< take all frequency components up to Freq_max with amplitude >= 'P1_min', even in the case that Wmin is satisfied

}; // class DMP_Shannon

} // namespace as64_

#endif // DYNAMICAL_MOVEMENT_PRIMITIVE_SHANNON_H
