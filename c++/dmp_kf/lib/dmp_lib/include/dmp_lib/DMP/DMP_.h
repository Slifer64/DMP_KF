/** DMP class
 *  Implements an 1-D this->
 * The DMP is driven by a canonical clock. It outputs the phase varialbe
 * 'x' which serves as a substitute for time. Typically, it evolves from
 * x0=0 at t=0 to x_end=1, at t=tau, where tau is the total movement's
 * duration. An example of a linear canonical clock is:
 *    dx = -ax/tau
 * where x is the phase variable and ax the evolution factor. Other types
 * of canonical clocks, such as exponential, can be used. However, keeping
 * a linear mapping between the phase variable 'x' and time 't' is more
 * intuitive.
 *
 * The DMP has the in general the following form:
 *
 *    tau*dz = g1(x)*( a_z*(b_z*(g-y) - z ) + g2(x)*fs*f(x) + z_c
 *    tau*dy = z + y_c;
 *
 * Assuming y_c=z_c=0, we can write equivalently:
 *    ddy = g1(x)*( a_z*(b_z*(g-y)-dy*tau) + g2(x)*fs*f(x) ) / tau^2;
 *
 * where
 *    tau: is scaling factor defining the duration of the motion
 *    a_z, b_z: constants relating to a spring-damper system
 *    fs: scaling of the forcing term (typically fs = g0-y0)
 *    g: the goal-final position
 *    y0: the initial position
 *    x: the phase variable
 *    y,dy,ddy: the position, velocity and accelaration of the motion
 *    f(x): the forcing term defined by the normalized weighted sum of the
 *       kernel functions (gaussian kernels), i.e.:
 *       f(x) = w'*Psi(x)/ sum(Psi(x));
 *    g1(x): the gating factor of the spring-damper term
 *    g2(x): the gating factor of non-linear forcing term
 */

#ifndef ABSTRACT_DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define ABSTRACT_DYNAMICAL_MOVEMENT_PRIMITIVE_H

#include <cmath>
#include <vector>
#include <cstring>
#include <iomanip>
#include <memory>
#include <exception>
#include <fstream>
#include <sstream>
#include <armadillo>

#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>
#include <param_lib/param_lib.h>

namespace as64_
{

class DMP_
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
  /** \brief DMP constructor.
   * @param[in] N_kernels the number of kernels.
   * @param[in] a_z Parameter 'a_z' relating to the spring-damper system.
   * @param[in] b_z Parameter 'b_z' relating to the spring-damper system.
   * @param[in] canClockPtr Pointer to a DMP canonical system object.
   * @param[in] shapeAttrGatingPtr Pointer to gating function for the shape attractor.
   * @param[in] goalAttrGatingPtr Pointer to gating function for the goal attractor.
   * @param[in] kernelStdScaling Scales the std of each kernel (optional, default = 1.0).
   * @param[in] extraArgName Names of extra arguments (optional, default = []).
   * @param[in] extraArgValue Values of extra arguemnts (optional, default = []).
   */
  DMP_(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr=NULL);

  DMP_(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    double kernel_std_scaling);

  DMP_();

  /** \brief Initializes the DMP
   * @param[in] N_kernels the number of kernels.
   * @param[in] a_z Parameter 'a_z' relating to the spring-damper system.
   * @param[in] b_z Parameter 'b_z' relating to the spring-damper system.
   * @param[in] canClockPtr Pointer to a DMP canonical system object.
   * @param[in] shapeAttrGatingPtr Pointer to gating function for the shape attractor.
   * @param[in] goalAttrGatingPtr Pointer to gating function for the goal attractor.
   * @param[in] kernelStdScaling Scales the std of each kernel (optional, default = 1).
   * @param[in] extraArgName Names of extra arguments (optional, default = []).
   * @param[in] extraArgValue Values of extra arguemnts (optional, default = []).
   */
  void init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr=NULL);

  void init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    double kernel_std_scaling);


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


  /** \brief Returns the scaling factor of the DMP.
   *  @return The scaling factor of the DMP.
   */
  double get_v_scale() const;

  /** \brief Sets the time cycle of the DMP.
   *  @param[in] tau The time duration for the DMP.
   */
  void setTau(double tau);

  /** \brief Returns the time cycle of the DMP.
   *  @return The time duration of the DMP.
   */
  double getTau() const;

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
  arma::vec getStatesDot(double x, double y, double z, double y0, double g, double y_c=0.0, double z_c=0.0) const;


  /** \brief Trains the DMP
   *  @param[in] Time Row vector with the timestamps of the training data points.
   *  @param[in] yd_data Row vector with the desired potition.
   *  @param[in] dyd_data Row vector with the desired velocity.
   *  @param[in] ddyd_data Row vector with the desired accelaration.
   *  @param[in] y0 Initial position.
   *  @param[in] g Target-goal position.
   *  @param[in] train_method Method to train the DMP wieghts (optinal, default = LWR).
   *  @param[in] ret_train_err If true calculates also the training error of the forcing term.
   *
   * \note The timestamps in \a Time and the corresponding position,
   * velocity and acceleration data in \a yd_data, \a dyd_data and \a
   * ddyd_data need not be sequantial in time.
   */
  virtual double train(const arma::rowvec &Time, const arma::rowvec &yd_data,
    const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double y0, double g,
    const std::string &train_method = "LWR", bool ret_train_err=false);


  /** \brief Sets the high level training parameters of the DMP.
   * @param[in] extraArgName Names of extra arguments (optional, default = []).
   * @param[in] extraArgValue Values of extra arguemnts (optional, default = []).
   *
   * \remark The extra argument names can be the following
   * 'lambda' Forgetting factor for recursive training methods.
   *  'P_cov' Initial value of the covariance matrix for recursive training methods.
   */
  virtual void setTrainingParams(const param_::ParamList *paramListPtr);


  /** \brief Calculates the desired values of the scaled forcing term.
   * @param[in] x The phase variable.
   * @param[in] y Position.
   * @param[in] dy Velocity.
   * @param[in] ddy Acceleration.
   * @param[in] y0 initial position.
   * @param[in] g Goal position.
   * @return Fd Desired value of the scaled forcing term.
   */
  virtual double calcFd(double x, double y, double dy, double ddy, double y0, double g) const;


  /** \brief Returns the learned forcing term.
   * @param[in] x The phase variable.
   * @param[in] y0 Initial position.
   * @param[in] g Goal position.
   * @return The learned forcing term.
   */
  virtual double learnedForcingTerm(double x, double y0, double g) const;


  /** \brief Returns the forcing term of the DMP
   * @param[in] x The phase variable.
   * @return The normalized weighted sum of Gaussians.
   */
  virtual double forcingTerm(double x) const;


  /** \brief Returns the scaling factor of the forcing term.
   * @param[in] y0 Initial position.
   * @param[in] g Goal position.
   * @return The scaling factor of the forcing term.
   */
  virtual double forcingTermScaling(double y0, double g) const;


  /** \brief Returns the goal attractor of the DMP.
   *  @param[in] x The phase variable.
   *  @param[in] y 'y' state of the DMP.
   *  @param[in] z 'z' state of the DMP.
   *  @param[in] g Goal position.
   *  @return The goal attractor of the DMP.
   */
  virtual double goalAttractor(double x, double y, double z, double g) const;


  /** \brief Returns the shape attractor of the DMP.
   *  @param[in] x The phase variable.
   *  @param[in] y0 Initial position.
   *  @param[in] g Goal position.
   *  @return The shape_attr of the DMP.
   */
  virtual double shapeAttractor(double x, double y0, double g) const;


  /** \brief Returns a column vector with the values of the kernel functions of the DMP.
   *  @param[in] x Phase variable.
   *  @return Column vector with the values of the kernel functions of the DMP.
   */
  virtual arma::vec kernelFunction(double x) const;

  /** \brief Sets the centers for the kernel functions of the DMP according to the canonical system.
   */
  void setCenters();


  /** \brief Sets the standard deviations for the kernel functions  of the DMP
   *  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
   *  @param[in] kernelStdScaling Scales the std of each kernel by 'kernelStdScaling' (optional, default = 1.0).
  */
  void setStds(double kernelStdScaling = 1.0);

  int getNumKernels() const;

  /** \brief Parse extra arguments of the DMP.
   *  @param[in] extraArgName Names of extra arguments.
   *  @param[in] extraArgValue Values of extra arguemnts.
   */
  virtual void parseExtraArgs(const param_::ParamList *paramListPtr);

  void update_weights_with_RLWR(double x, double Ferr, double y0, double g, arma::mat &Sigma_w, double lambda);

  void update_weights_with_KF(double x, double Ferr, double y0, double g, arma::mat &Sigma_w, double sigma_noise);

  void update_weights_with_RLS(double x, double Ferr, double y0, double g, arma::mat &Sigma_w, double lambda);

protected:
  std::shared_ptr<CanonicalClock> canClockPtr; ///< pointer to the canonical clock
  std::shared_ptr<GatingFunction> shapeAttrGatingPtr; ///< pointer to gating function for the shape attractor
  std::shared_ptr<GatingFunction> goalAttrGatingPtr; ///< pointer to gating function for the goal attractor

  long double zero_tol; ///< tolerance value used to avoid divisions with very small numbers
  double a_s; ///< scaling factor to ensure smaller changes in the accelaration to improve the training
  double kernelStdScaling; ///< scaling factor for the kernels std

  // training params
  double lambda; ///< forgetting factor in recursive training methods
  double P_cov; ///< Initial value of covariance matrix in recursive training methods

}; // class DMP_

} // namespace as64_

#endif // ABSTRACT_DYNAMICAL_MOVEMENT_PRIMITIVE_H
