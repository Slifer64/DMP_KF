#include <dmp_lib/DMP/DMP_bio.h>


namespace as64_
{

  DMP_bio::DMP_bio() {}

  DMP_bio::DMP_bio(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr)
  {
    this->init(N_kernels, a_z, b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, paramListPtr);
  }

  double DMP_bio::calcFd(double x, double y, double dy, double ddy, double y0, double g) const
  {
    double u = this->shapeAttrGating(x);
    double v_scale = this->get_v_scale();
    double K = this->a_z * this->b_z;
    double Fd = (ddy*std::pow(v_scale,2) - this->goalAttractor(x, y, v_scale*dy, g) + K*(g-y0)*u);
    return Fd;
  }

  double DMP_bio::forcingTermScaling(double y0, double g) const
  {
    double K = this->a_z*this->b_z;
    double f_scale = K;
    return f_scale;
  }

  double DMP_bio::shapeAttractor(double x, double y0, double g) const
  {
    double K = this->a_z*this->b_z;
    double s_attr_gating = this->shapeAttrGating(x);
    double f = this->forcingTerm(x);
    double f_scale = this->forcingTermScaling(y0, g);
    double shape_attr = s_attr_gating * (f * f_scale - K*(g-y0));
    return shape_attr;
  }

} // namespace as64_
