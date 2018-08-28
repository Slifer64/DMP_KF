#ifndef DMP_UTILS_H
#define DMP_UTILS_H

#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/CanonicalClock/LinCanonicalClock.h>

#include <dmp_lib/GatingFunction/GatingFunction.h>
#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/ConstGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>
#include <dmp_lib/GatingFunction/SpringDamperGatingFunction.h>

#include <dmp_lib/DMP/DMP_.h>
#include <dmp_lib/DMP/DMP.h>
#include <dmp_lib/DMP/DMP_bio.h>
#include <dmp_lib/DMP/DMP_plus.h>
#include <dmp_lib/DMP/DMP_Shannon.h>
#include <dmp_lib/DMP/DMP_VT.h>

namespace as64_
{

std::shared_ptr<CanonicalClock> getCanClock(const std::string &CAN_CLOCK_TYPE, double tau);


std::shared_ptr<DMP_> getDMP(const std::string &DMP_TYPE, int N_kernels, double a_z, double b_z,
                            std::shared_ptr<CanonicalClock> canClockPtr,
                            std::shared_ptr<GatingFunction> shapeAttrGatingPtr,
                            std::shared_ptr<GatingFunction> goalAttrGatingPtr,
                            const param_::ParamList *paramListPtr=NULL);


std::shared_ptr<GatingFunction> getGatingFun(const std::string &GATTING_FUN_TYPE, double u0, double u_end);

} // namespace as64_

#endif // DMP_UTILS_H
