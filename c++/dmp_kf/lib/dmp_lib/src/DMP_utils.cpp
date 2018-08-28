#include <dmp_lib/DMP_utils.h>

namespace as64_
{

std::shared_ptr<CanonicalClock> getCanClock(const std::string &CAN_CLOCK_TYPE, double tau)
{
  std::shared_ptr<CanonicalClock> canClockPtr;
  if (CAN_CLOCK_TYPE.compare("lin")==0)
  {
    canClockPtr.reset(new LinCanonicalClock());
  }
  else
  {
    throw std::runtime_error(std::string("Unsupported canonical clock type \"") + CAN_CLOCK_TYPE + "\".\n");
  }
  canClockPtr->init(tau);

  return canClockPtr;
}


std::shared_ptr<DMP_> getDMP(const std::string &DMP_TYPE, int N_kernels, double a_z, double b_z,
                            std::shared_ptr<CanonicalClock> canClockPtr,
                            std::shared_ptr<GatingFunction> shapeAttrGatingPtr,
                            std::shared_ptr<GatingFunction> goalAttrGatingPtr,
                            const param_::ParamList *paramListPtr)
{
  std::shared_ptr<DMP_> dmp;

  if (DMP_TYPE.compare("DMP")==0)
  {
      dmp.reset(new DMP());
  }
  else if (DMP_TYPE.compare("DMP-bio")==0)
  {
      dmp.reset(new DMP_bio());
  }
  else if (DMP_TYPE.compare("DMP-plus")==0)
  {
      dmp.reset(new DMP_plus());
  }
  else if (DMP_TYPE.compare("DMP-Shannon")==0)
  {
      dmp.reset(new DMP_Shannon());
  }
  else
  {
    throw std::runtime_error(std::string("Unsupported DMP type \"") + DMP_TYPE + "\".\n");
  }

  dmp->init(N_kernels, a_z, b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, paramListPtr);

  return dmp;
}

std::shared_ptr<GatingFunction> getGatingFun(const std::string &GATTING_FUN_TYPE, double u0, double u_end)
{

  std::shared_ptr<GatingFunction> gatingFunPtr;

  if (GATTING_FUN_TYPE.compare("lin")==0)
  {
    gatingFunPtr.reset(new LinGatingFunction());
  }
  else if (GATTING_FUN_TYPE.compare("exp")==0)
  {
    gatingFunPtr.reset(new ExpGatingFunction());
  }
  else if (GATTING_FUN_TYPE.compare("sigmoid")==0)
  {
    gatingFunPtr.reset(new SigmoidGatingFunction());
  }
  else if (GATTING_FUN_TYPE.compare("spring-damper")==0)
  {
    gatingFunPtr.reset(new SpringDamperGatingFunction());
  }
  else if (GATTING_FUN_TYPE.compare("constant")==0)
  {
    gatingFunPtr.reset(new ConstGatingFunction());
  }
  else
  {
      throw std::runtime_error(std::string("Unsupported gating function type \"") + GATTING_FUN_TYPE + "\".\n");
  }

  gatingFunPtr->init(u0, u_end);

  return gatingFunPtr;
}

} // namespace as64_
