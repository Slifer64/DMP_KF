/** DMP Cartesian Position class
 *  Implements an 3-D DMP representing the Cartesian position.
 *  The DMP is driven by a canonical clock. It outputs the phase varialbe
 *  'x' which serves as a substitute for time. Typically, it evolves from
 *  x0=0 at t=0 to x_end=1, at t=tau, where tau is the total movement's
 *  duration. An example of a linear canonical clock is:
 *     dx = -ax/tau
 *  where x is the phase variable and ax the evolution factor. Other types
 *  of canonical clocks, such as exponential, can be used. However, keeping
 *  a linear mapping between the phase variable 'x' and time 't' is more
 *  intuitive.
 *
 *  The DMP has the in general the following form:
 *
 *     tau*dz = g1(x)*( a_z*(b_z*(g-y) - z ) + g2(x)*fs*f(x) + z_c
 *     tau*dy = z + y_c;
 *
 *  Assuming y_c=z_c=0, we can write equivalently:
 *     ddy = g1(x)*( a_z*(b_z*(g-y)-dy*tau) + 2(x)*fs*f(x) ) / tau^2;
 *
 *  where
 *     tau: is scaling factor defining the duration of the motion
 *     a_z, b_z: constants relating to a spring-damper system
 *     fs: scaling of the forcing term (typically fs = g0-y0)
 *     g: the goal-final position
 *     y0: the initial position
 *     x: the phase variable
 *     y,dy,ddy: the position, velocity and accelaration of the motion
 *     f(x): the forcing term defined by the normalized weighted sum of the
 *        kernel functions (gaussian kernels), i.e.:
 *        f(x) = w'*Psi(x)/ sum(Psi(x));
 *     g1(x): the gating factor of the spring-damper term
 *     g2(x): the gating factor of non-linear forcing term
 */

#ifndef CARTPOS_DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define CARTPOS_DYNAMICAL_MOVEMENT_PRIMITIVE_H

#include <cmath>
#include <vector>
#include <cstring>
#include <iomanip>
#include <memory>
#include <exception>
#include <armadillo>

#include <dmp_lib/DMP/DMP_.h>
#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>
#include <param_lib/param_lib.h>

namespace as64_
{

class DMP_CartPos
{
  // properties
public:
  std::vector<std::shared_ptr<DMP_>> dmp; ///< vector 3x1 of 1-D DMPs
  const int D; ///< dimensionality of the DMP_CartPos (= 3, constant)

  // methods
public:
  /** brief DMP empty constructor.
   */
  DMP_CartPos();

  /** brief DMP constructor.
   *  @param[in] vec3D_dmp 3x1 cell array of 1D DMPs.
   */
  DMP_CartPos(std::vector<std::shared_ptr<DMP_>> &vec3D_dmp);


  /** brief Initializes the DMP.
   *  @param[in] vec3D_dmp: 3x1 cell array of 1D DMPs.
   */
  void init(std::vector<std::shared_ptr<DMP_>> &vec3D_dmp);


  /** brief Sets the centers for the kernel voids of the DMP according to the canonical system
   */
  void setCenters();


  /** brief Sets the standard deviations for the kernel voids  of the DMP
   * Sets the variance of each kernel equal to squared difference between the current and the next kernel.
   * @param[in] kernelStdScaling: Scales the variance of each kernel by 'kernelStdScaling' (optional, default = 1.0).
  */
  void setStds(double kernelStdScaling = 1.0);


  /** brief Trains the DMP
   * @param[in] Time: Row vector with the timestamps of the training data points.
   * @param[in] Y_data: Matrix with the Cartesian position in each column.
   * @param[in] dY_data: Matrix with the Cartesian velocity in each column.
   * @param[in] ddY_data: Matrix with the Cartesian acceleration in each column.
   * @param[in] Y0: Initial Cartesian position.
   * @param[in] Yg: Target-goal Cartesian position.
   * @param[in] train_method Method to train the DMP wieghts (optinal, default = LWR).
   * @param[in] ret_train_err If true calculates also the training error of the forcing term.
   *
   * \note The timestamps in \a Time and the corresponding position,
   * velocity and acceleration data in \a Y_data, \a dY_data and \a
   * ddY_data need not be sequantial in time.
   */
  arma::vec train(const arma::rowvec &Time, const arma::mat &Y_data,
                  const arma::mat &dY_data, const arma::mat &ddY_data,
                  const arma::vec &Y0, const arma::vec &Yg,
                  const std::string &train_method = "LWR", bool ret_train_err=false);


  /** brief Sets the high level training parameters of the DMP
   * @param[in] trainMethod: Method used to train the DMP weights.
   * @param[in] extraArgName: Names of extra arguments (optional, default = []).
   * @param[in] extraArgValue: Values of extra arguemnts (optional, default = []).
   *
   * \remark The extra argument names can be the following:
   * 'lambda': Forgetting factor for recursive training methods.
   * 'P_cov': Initial value of the covariance matrix for recursive training methods.
   */
  void setTrainingParams(const param_::ParamList *paramListPtr);


  /** brief Calculates the desired values of the scaled forcing term.
   * @param[in] X: 3x1 vector with the phase variable of each DMP.
   * @param[in] Y: Cartesian position.
   * @param[in] dY: Cartesian velocity.
   * @param[in] ddY: Cartesian acceleration.
   * @param[in] Y0: Initial Cartesian position.
   * @param[in] Yg: Goal Cartesian position.
   * @param[out] Fd: Desired value of the scaled forcing term.
   */
  arma::vec calcFd(const arma::vec &X, const arma::vec &Y, const arma::vec &dY,
                  const arma::vec &ddY, const arma::vec &Y0, const arma::vec &Yg) const;


  arma::vec learnedForcingTerm(const arma::vec &X, const arma::vec &y0, const arma::vec &g) const;


  /** brief Returns the forcing term of the DMP.
   * @param[in] X: 3x1 vector with the phase variable of each DMP.
   * @param[out] f: The normalized weighted sum of Gaussians.
   */
  arma::vec forcingTerm(const arma::vec &X) const;

  /** brief Returns the scaling factor of the forcing term.
   * @param[in] Y0: Initial Cartesian position.
   * @param[in] Yg: Goal Cartesian position.
   * @param[out] f_scale: The scaling factor of the forcing term.
   */
  arma::vec forcingTermScaling(const arma::vec &Y0, const arma::vec &Yg) const;

  /** brief Returns the goal attractor of the DMP.
   * @param[in] X: 3x1 vector with the phase variable of each DMP.
   * @param[in] Y: \a y state of the DMP.
   * @param[in] Z: \a z state of the DMP.
   * @param[in] Yg: Goal Cartesian position.
   * @param[out] goal_attr: The goal attractor of the DMP.
   */
  arma::vec goalAttractor(const arma::vec &X, const arma::vec &Y, const arma::vec &Z, const arma::vec &Yg) const;


  /** brief Returns the shape attractor of the DMP.
   * @param[in] X: 3x1 vector with the phase variable of each DMP.
   * @param[in] Y0: Initial Cartesian position.
   * @param[in] Yg: Goal Cartesian position.
   * @param[out] shape_attr: The shape_attr of the DMP.
   */
  arma::vec shapeAttractor(const arma::vec &X, const arma::vec &Y0, const arma::vec &Yg) const;


  /** brief Returns the derivatives of the DMP states
   * @param[in] X: 3x1 vector with the phase variable of each DMP.
   * @param[in] Y: \a y state of the DMP.
   * @param[in] Z: \a z state of the DMP.
   * @param[in] Y0: Initial position.
   * @param[in] Yg: Goal position.
   * @param[in] Y_c: Coupling term for the dynamical equation of the \a y state.
   * @param[in] Z_c: Coupling term for the dynamical equation of the \a z state.
   * @param[out] dY: Derivative of the \a y state of the DMP.
   * @param[out] dZ: Derivative of the \a z state of the DMP.
   * @param[out] dX: Derivative of the state variable.
   */
  arma::mat getStatesDot(const arma::vec &X, const arma::vec &Y, const arma::vec &Z,
                              const arma::vec &Y0, const arma::vec &Yg,
                              const arma::vec &Y_c, const arma::vec &Z_c) const;


  /** brief Returns a column vector with the values of the kernel voids of the DMP
   * @param[in] X: 3x1 vector with the phase variable of each DMP.
   * @param[out] Psi: 3x1 cell array of column vectors with the values of the kernel voids of each DMP.
   */
  std::vector<arma::vec> kernelFunction(const arma::vec &X) const;


  /** brief Returns the scaling factor of the DMP
   * @param[out] v_scale: 3x1 vector with the scaling factor of each DMP.
   */
  arma::vec get_v_scale() const;


  /** brief Returns the time cycle of the DMP
   * @param[out] tau: 3x1 vector with the time duration of each DMP.
   */
  arma::vec getTau() const;

  arma::vec phase(double t) const;

}; // class DMP_

} // namespace as64_

#endif // CARTPOS_DYNAMICAL_MOVEMENT_PRIMITIVE_H
