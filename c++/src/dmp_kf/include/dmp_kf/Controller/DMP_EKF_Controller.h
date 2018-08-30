#ifndef DMP_EKF_CONTROLLER_H
#define DMP_EKF_CONTROLLER_H

#include <cstdlib>
#include <vector>
#include <memory>
#include <exception>
#include <cstring>

#include <armadillo>

#include <dmp_lib/dmp_lib.h>
#include <dmp_kf/Controller/Controller.h>


class DMP_EKF_Controller: public Controller
{
public:
  DMP_EKF_Controller(std::shared_ptr<Robot> &robot);
  ~DMP_EKF_Controller() {}

  void initExecution();
  void run();

  void initTraining();
  void logTrainData();
  void train();

private:

  void readExecutionParams(const char *params_file = NULL);
  bool startExecution();

  void readTrainingParams(const char *params_file = NULL);

  // =========  model  =========
  std::vector<std::shared_ptr<as64_::DMP>> dmp;
  std::shared_ptr<as64_::CanonicalClock> can_clock_ptr;
  std::shared_ptr<as64_::GatingFunction> shape_attr_gating_ptr;
  int N_kernels;
  double a_z, b_z;
  std::string train_method;

  // =========  KF params  =========
  // arma::vec g_hat; // goal estimate
  // double tau_hat; // time scale estimate
  arma::vec theta; // theta = [g_hat; tau_hat]
  // arma::mat P_theta; // covariance of g_hat and tau_hat
  double x_hat;
  arma::vec P0_g_hat; // initial covariance for the goal
  double P0_tau_hat; // initial covariance for the time scale
  arma::mat R_v, inv_R_v; // measurement noise variance matrix
  double a_p; // instability term for P_dot in modified EKF
  arma::mat Q_w; // process noise variance matrix

  // Training data for the DMP
  double t_d; // current timestamp during demo recording
  // arma::rowvec Timed; // timestamps from demo
  // arma::mat Yd_data, dYd_data, ddYd_data;
  arma::vec p, p_prev, dp, dp_prev, ddp; // store current and previous positions and velocities for numerical diff
  arma::vec g_d; // goal position from demo used for initializing g_hat
  double tau_d; // time scale from demo used for initializing tau_hat
  double a_filt; // Coefficient of LP filter: y_dot = (1-a_filt)*y_dot + a_filt*(y-y_prev)/dt

  // ========= Controller params ===========
  arma::mat M; ///< impedance controller inertia matrix
  arma::mat K; ///< impedance controller stiffness matrix
  arma::mat D; ///< impedance controller damping matrix
  arma::vec U_dmp; ///< control produced from dmp ref_model
  arma::vec U_total; ///< U_dmp + feedforward force
  double k_click; // click

  // double t; // current timestamp during controller execution
  arma::vec Y0; // initial position for execution
  // arma::vec Y, dY, ddY; // produced by the target impedance model
  arma::vec Y_ref, dY_ref, ddY_ref; // produced by the DMP
  // arma::vec f_ext; // external Cartesian force
  arma::vec ff_gains; // feedforward force gains for target impedance
  double a_force; // iir 1st order filter coeff for filtering force measurements

  // starting conditions
  bool start_flag; // if true run() can start
  double f_thres; // if norm(f_ext) > f_thres then the controller can start running

  // leader-follower sigmoid function params  1 / ( 1 + exp(a_m*(norm(f_ext)-c_m)) )
  double a_m; // sigmoid steepness
  double c_m; // sigmoid center
};

#endif // DMP_EKF_CONTROLLER_H
