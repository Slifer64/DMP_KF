/** DMP_VT class
 *  Velocity tracking DMP
 */

#ifndef VELOCITY_TRACK_DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define VELOCITY_TRACK_DYNAMICAL_MOVEMENT_PRIMITIVE_H

#include <cmath>
#include <vector>
#include <cstring>
#include <iomanip>
#include <memory>
#include <exception>
#include <fstream>
#include <sstream>
#include <armadillo>

#include <dmp_lib/DMP/DMP_.h>

namespace as64_
{

class DMP_VT: public DMP_
{
  // properties
public:
  int N_kernels; ///< number of kernels (basis functions)

  double a_z; ///< parameter 'a_z' relating to the spring-damper system
  double b_z; ///< parameter 'b_z' relating to the spring-damper system

  arma::vec w; ///< N_kernelsx1 vector with the weights of the DMP
  arma::vec c; ///< N_kernelsx1 vector with the kernel centers of the DMP
  arma::vec h; ///< N_kernelsx1 vector with the kernel stds of the DMP

  // methods
public:

  DMP_VT(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr=NULL);

  DMP_VT(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    double kernel_std_scaling);

  DMP_VT();

  virtual double forcingTermScaling(double y0, double g) const;

protected:

}; // class DMP_VT

} // namespace as64_

#endif // VELOCITY_TRACK_DYNAMICAL_MOVEMENT_PRIMITIVE_H
