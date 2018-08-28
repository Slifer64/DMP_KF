#ifndef DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define DYNAMICAL_MOVEMENT_PRIMITIVE_H

#include <dmp_lib/DMP/DMP_.h>

namespace as64_
{

class DMP: public DMP_
{
public:
  DMP();

  DMP(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr=NULL);

}; // class DMP

} // namespace as64_

#endif // DYNAMICAL_MOVEMENT_PRIMITIVE_H
