#include <dmp_lib/DMP/DMP.h>


namespace as64_
{

  DMP::DMP() {}

  DMP::DMP(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr)
  {
    this->init(N_kernels, a_z, b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, paramListPtr);
  }


} // namespace as64_
