#include <dmp_lib/DMP/DMP_VT.h>

namespace as64_
{

  DMP_VT::DMP_VT(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr)
  {
    this->init(N_kernels, a_z, b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, paramListPtr);
  }

  DMP_VT::DMP_VT(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    double kernel_std_scaling)
  {
    this->init(N_kernels, a_z, b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, kernel_std_scaling);
  }

  DMP_VT::DMP_VT() {}

  double DMP_VT::forcingTermScaling(double y0, double g) const
  {
    double f_scale = 1.0;
    return f_scale;
  }

} // namespace as64
