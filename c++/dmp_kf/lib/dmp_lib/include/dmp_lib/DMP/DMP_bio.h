#ifndef DYNAMICAL_MOVEMENT_PRIMITIVE_BIO_H
#define DYNAMICAL_MOVEMENT_PRIMITIVE_BIO_H

#include <dmp_lib/DMP/DMP_.h>

namespace as64_
{

class DMP_bio: public DMP_
{
public:
  DMP_bio();

  DMP_bio(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr=NULL);

  virtual double calcFd(double x, double y, double dy, double ddy, double y0, double g) const;

  virtual double forcingTermScaling(double y0, double g) const;

  virtual double shapeAttractor(double x, double y0, double g) const;

}; // class DMP_bio

} // namespace as64_

#endif // DYNAMICAL_MOVEMENT_PRIMITIVE_BIO_H
