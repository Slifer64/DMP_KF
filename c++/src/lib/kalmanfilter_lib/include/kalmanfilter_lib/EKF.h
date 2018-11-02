/** EKF class
 * Implementation of the discrete Extended Kalman Filter algorithm.
 * - The Jacobian of the state transition and measurement functions can either
 * be provided or approximated numerically.
 * - A fading memory coefficient can be defined.
 * - Enforcement of linear constraints can be imposed on the estimated parameteres.
 */

#ifndef EXTENDED_KALMAN_FILTER_AS64_H
#define EXTENDED_KALMAN_FILTER_AS64_H

#include <cmath>
#include <armadillo>
#include <Eigen/Dense>

namespace as64_
{

namespace kf_
{

class EKF
{
public:
  /** \brief Extended Kalman Filter constructor.
   * @param[in] N_params Number of states to estimate.
   * @param[in] N_out Number of outputs.
   */
  EKF(unsigned N_params, unsigned N_out);

  /** \brief Sets the fading memory coefficient.
   *  \note If the continuous time fading memory coefficient is 'a' then
   *  the discrete one is 'a_p = exp(a*Ts)' where Ts is the sampling period.
   * @param[in] a_p: Fading memory coefficient.
   */
  void setFadingMemoryCoeff(double a_p);

  /** \brief Enables/Disables constraints in the estimation parameters.
   *  \note The constraints must be set with 'setParamsConstraints' first.
   *  @param[in] enable_contraints Flag that is true/false for enabling/disabling the constraints.
   */
  void enableParamsContraints(bool enable_contraints);

  /** \brief Sets linear constraints in the estimation parameters.
   * The constraints are such that D_up*theta <= d_up and D_low*theta >= d_low
   * @param[in] D_up Constraint matrix for upper bound constraints.
   *  @param[in] d_up Limits for upper bound constraints.
   *  @param[in] D_low Constraint matrix for lower bound constraints.
   *  @param[in] d_low Limits for lower bound constraints.
   */
  void setParamsConstraints(const arma::mat &D_up, const arma::vec &d_up, const arma::mat &D_low, const arma::vec &d_low);

  /** \brief Sets the covariance matrix of the process noise.
   * \note If the continuous time process noise is R_c the discrete
   * one is 'R = Q_c/Ts' where Ts is the sampling period.
   * @param[in] Q Process noise covariance matrix.
   */
  void setProcessNoiseCov(const arma::mat &Q);

  /** \brief Sets the covariance matrix of the measurement noise.
   *  \note that if the continuous time process noise is Q_c the discrete
   *  one is 'Q = Q_c*Ts' where Ts is the sampling period.
   * @param[in] R Measurement noise covariance matrix.
   */
  void setMeasureNoiseCov(const arma::mat &R);

  /** \brief Sets the step for computing the partial derivatives of the state
   * transition and/or measurement function w.r.t. the estimation parameters.
   * @param[in] dtheta Vector of step size for each parameter.
   */
  void setPartDerivStep(const arma::vec &dtheta);

  /** \brief Performs the EKF prediction (time update).
   * @params[in] F_k Jacobian matrix of the state transition function w.r.t to the estimation parameters.
   */
  void predict(const arma::mat &F_k);

  /** \brief Performs the EKF prediction (time update) using numerical differentiation
   * for approximating the Jacobian of the state transition matrix w.r.t. the estimation parameters.
   * The step size for numerical differentiation can be set from 'setPartDerivStep'.
   * @param[in] state_trans_fun_ptr Pointer to state transition function. It should accept a
   *                                  vector (the parameters) and void pointer (cookie).
   * @param[in] cookie Pointer that is passed to 'state_trans_fun_ptr' and can contain extra
   *                     arguments needed. If not needed it can be set to NULL.
   */
  void predictApprox(void (*state_trans_fun_ptr)(const arma::vec &, void *), void *cookie);

  /** \brief Performs the EKF correction (measurement update).
   * @param[in] z Groundtruth measurements.
   * @param[in] z_hat Estimated measurements.
   * @param[in] H_k Jacobian of the measurement function w.r.t to the estimation parameters.
   */
  void correct(const arma::vec &z, const arma::vec &z_hat, const arma::mat &H_k);

  /** \brief Performs the EKF prediction (time update) using numerical differentiation
   * for approximating the Jacobian of the state transition matrix w.r.t. the estimation parameters.
   *  The step size for numerical differentiation can be set from 'setPartDerivStep'.
   * @param[in] state_trans_fun_ptr Pointer to state transition function. It should accept a
   *                                  vector (the parameters) and void pointer (cookie).
   * @param[in] cookie: Pointer that is passed to 'state_trans_fun_ptr' and can contain extra
   *                     arguments needed. If not needed it can be set to NULL.
   */
  void correctApprox(void (*measure_trans_fun_ptr)(const arma::vec &, void *), void *cookie);

  arma::vec theta; ///< params estimation
  arma::mat P; ///< params estimation covariance

private:

  arma::mat F_k; ///< forward model
  arma::mat H_k; ///< measurement model
  arma::mat K; ///< Kalman gain
  arma::mat Q; ///< process covariance
  arma::mat R; ///< measurement covariance

  double a_p; ///< fading memory coefficient

  // Apply projection so that:
  // D_up * theta <= d_up
  // D_low * theta >= d_low
  bool enable_constraints;
  arma::mat D_up; ///< upper bound constraint matrix
  arma::vec d_up; ///< upper bound constraints
  arma::mat D_low; ///< lower bound constraint matrix
  arma::vec d_low; ///< lower bound constraints

  bool apply_cov_sat;
  arma::vec theta_sigma_min;
  arma::vec theta_sigma_max;

  double dtheta; ///< Parameters step size for numerical approximation of transition and measurement function Jacobian

};

} // namespace kf_

} // namespace as64_

#endif // EXTENDED_KALMAN_FILTER_AS64_H
