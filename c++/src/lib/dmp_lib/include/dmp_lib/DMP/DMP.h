#ifndef DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define DYNAMICAL_MOVEMENT_PRIMITIVE_H

#include <cmath>
#include <vector>
#include <cstring>
#include <memory>
#include <exception>
#include <armadillo>

#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

class DMP
{
  // properties
public:
  int N_kernels; ///< number of kernels (basis functions)

  double a_z; ///< parameter 'a_z' relating to the spring-damper system
  double b_z; ///< parameter 'b_z' relating to the spring-damper system

  arma::vec w; ///< N_kernels x 1 vector with the kernels' weights
  arma::vec c; ///< N_kernels x 1 vector with the kernels' centers
  arma::vec h; ///< N_kernels x 1 vector with the kernels' inverse width

  std::shared_ptr<CanonicalClock> can_clock_ptr; ///< pointer to the canonical clock
  std::shared_ptr<GatingFunction> shape_attr_gating_ptr; ///< pointer to gating function for the shape attractor

  long double zero_tol; ///< small value used to avoid divisions with very small numbers

  // methods
public:

  /** \brief DMP constructor.
   * @param[in] N_kernels The number of kernels.
   * @param[in] a_z Parameter 'a_z' relating to the spring-damper system.
   * @param[in] b_z Parameter 'b_z' relating to the spring-damper system.
   * @param[in] can_clock_ptr Pointer to a DMP canonical system object.
   * @param[in] shape_attr_gating_ptr Pointer to gating function for the shape attractor.
   */
  DMP(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> &can_clock_ptr,
    std::shared_ptr<GatingFunction> &shape_attr_gating_ptr);

  /** \brief Initializes the DMP
   * @param[in] N_kernels The number of kernels.
   * @param[in] a_z Parameter 'a_z' relating to the spring-damper system.
   * @param[in] b_z Parameter 'b_z' relating to the spring-damper system.
   * @param[in] can_clock_ptr Pointer to a DMP canonical system object.
   * @param[in] shape_attr_gating_ptr Pointer to gating function for the shape attractor.
   */
  void init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> &can_clock_ptr,
    std::shared_ptr<GatingFunction> &shape_attr_gating_ptr);


  /** \brief Trains the DMP weights of the kernels.
   *  @param[in] train_method Method to train the DMP weights ('LWR' or 'LS').
   *  @param[in] Time Row vector with the timestamps of the training data points.
   *  @param[in] yd_data Row vector with the desired potition.
   *  @param[in] dyd_data Row vector with the desired velocity.
   *  @param[in] ddyd_data Row vector with the desired accelaration.
   *  @param[in] ret_train_err Flag to return the training error (optinal, default = false).
   *  @return The training error (-1 is returned if \a ret_train_err=false)
   */
  double train(const std::string &train_method, const arma::rowvec &Time, const arma::rowvec &yd_data,
    const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, bool ret_train_err=false);


  /** \brief Returns the derivatives of the DMP states.
   *  @param[in] x phase variable.
   *  @param[in] y \a y state of the DMP.
   *  @param[in] z \a z state of the DMP.
   *  @param[in] y0 Initial position.
   *  @param[in] g Goal position.
   *  @param[in] y_c Coupling term for the dynamical equation of the \a y state.
   *  @param[in] z_c Coupling term for the dynamical equation of the \a z state.
   *  @return  The states derivatives of the DMP as a 3x1 vector (dz, dy, dx).
   */
  arma::vec statesDot(double x, double y, double z, double y0, double g, double y_c=0.0, double z_c=0.0) const;


  /** \brief Returns the number of kernels of the DMP.
   * @return Number of kernels of the DMP.
   *
   */
  int numOfKernels() const;


  /** \brief Returns the phase variable.
   *  @param[in] t The time instant.
   *  @return The phase variable for time 't'.
   */
  double phase(double t) const;


  /** \brief Returns the derivative of the phase variable.
   *  @param[in] x The phase variable.
   *  @return The derivative of the phase variable.
   */
  double phaseDot(double x) const;


  /** \brief Returns a column vector with the values of the kernel functions of the DMP.
   *  @param[in] x Phase variable.
   *  @return Column vector with the values of the kernel functions of the DMP.
   */
  arma::vec kernelFunction(double x) const;


  /** \brief Sets the time scale of the DMP.
   *  @param[in] tau The time duration for the DMP.
   */
  void setTau(double tau);

  /** \brief Returns the time scale of the DMP.
   *  @return The time scale of the DMP.
   */
  double getTau() const;

  /** \brief Returns the partial derivative of the DMP's acceleration wrt to the goal and tau.
   *  @param[in] t Current timestamp.
   *  @param[in] y Position.
   *  @param[in] dy Velocity.
   *  @param[in] y0 Initial position.
   *  @param[in] x_hat Phase variable estimate.
   *  @param[in] g_hat Goal estimate.
   *  @param[in] tau_hat Time scale estimate.
   *  @return Partial derivative of the DMP's acceleration wrt to the goal and tau.
   */
  arma::vec getAcellPartDev_g_tau(double t, double y, double dy, double y0,
                                  double x_hat, double g_hat, double tau_hat) const;

  /** \brief Returns the DMP's acceleration.
   * @param[in] y Position.
   * @param[in] dy Velocity.
   * @param[in] y0 Initial position.
   * @param[in] y_c Coupling term for the dynamical equation of the \a y state.
   * @param[in] z_c Coupling term for the dynamical equation of the \a z state.
   * @param[in] x_hat Phase variable estimate.
   * @param[in] g_hat Goal estimate.
   * @param[in] tau_hat Time scale estimate.
   * @return ddy DMP's acceleration.
   */
  double getAccel(double y, double dy, double y0, double y_c, double z_c,
                  double x_hat, double g_hat, double tau_hat) const;

private:

  /** \brief Returns the goal attractor of the DMP.
   *  @param[in] x The phase variable.
   *  @param[in] y 'y' state of the DMP.
   *  @param[in] z 'z' state of the DMP.
   *  @param[in] g Goal position.
   *  @return The goal attractor of the DMP.
   */
  double goalAttractor(double x, double y, double z, double g) const;


  /** \brief Returns the shape attractor of the DMP.
   *  @param[in] x The phase variable.
   *  @param[in] y0 Initial position.
   *  @param[in] g Goal position.
   *  @return The shape_attr of the DMP.
   */
  double shapeAttractor(double x, double y0, double g) const;


  /** \brief Returns the shape attractor gating factor.
   *  @param[in] x The phase variable.
   *  @return The shape attractor gating factor.
   */
  double shapeAttrGating(double x) const;


  /** \brief Returns the goal attractor gating factor.
   *  @param[in] x The phase variable.
   *  @return The goal attractor gating factor.
   */
  double goalAttrGating(double x) const;


  /** \brief Returns the forcing term of the DMP
   * @param[in] x The phase variable.
   * @return The normalized weighted sum of Gaussians.
   */
  double forcingTerm(double x) const;


  /** \brief Returns the scaling factor of the forcing term.
   * @param[in] y0 Initial position.
   * @param[in] g Goal position.
   * @return The scaling factor of the forcing term.
   */
  double forcingTermScaling(double y0, double g) const;


  /** \brief Sets the centers for the kernel functions of the DMP according to the canonical system.
   */
  void setCenters();


  /** \brief Sets the standard deviations for the kernel functions  of the DMP
   *  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
   *  @param[in] kernel_std_scaling Scales the std of each kernel by 'kernelStdScaling' (optional, default = 1.0).
  */
  void setStds(double kernel_std_scaling = 1.0);


  /** \brief Calculates the desired values of the scaled forcing term.
   * @param[in] x The phase variable.
   * @param[in] y Position.
   * @param[in] dy Velocity.
   * @param[in] ddy Acceleration.
   * @param[in] y0 initial position.
   * @param[in] g Goal position.
   * @return Fd Desired value of the scaled forcing term.
   */
  double calcFd(double x, double y, double dy, double ddy, double y0, double g) const;

}; // class DMP

} // namespace as64_

#endif // DYNAMICAL_MOVEMENT_PRIMITIVE_H
