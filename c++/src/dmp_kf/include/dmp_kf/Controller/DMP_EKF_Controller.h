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

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

#include <kalmanfilter_lib/EKF.h>


class DMP_EKF_Controller: public Controller
{
public:
  DMP_EKF_Controller(std::shared_ptr<Robot> &robot, std::shared_ptr<GUI> &gui);
  ~DMP_EKF_Controller() {}

  void initExecution();
  void execute();

  bool simulate();

  void initDemo();
  void logDemoData();
  bool train(std::string &err_msg);

  bool loadTrainedModel(std::string &err_msg);
  bool saveTrainedModel(std::string &err_msg);

  bool runModel();

private:

  arma::vec msrFun(const arma::vec &theta, void *cookie=NULL);
  arma::vec stateTransFun(const arma::vec &theta, void *cookie=NULL);

  arma::mat msrFunJacob(const arma::vec &theta, void *cookie=NULL);
  arma::mat stateTransFunJacob(const arma::vec &theta, void *cookie=NULL);

  void initEKF();
  void initVariables();

  std::shared_ptr<as64_::kf_::EKF> ekf;

  void readControllerParams(const char *params_file = NULL);
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
  arma::vec P0_g_hat; // initial covariance for the goal
  double P0_tau_hat; // initial covariance for the time scale
  arma::mat R_v, inv_R_v; // measurement noise variance matrix
  double a_p; // fading memory coefficient in EKF
  arma::mat Q_w; // process noise variance matrix
  arma::mat A_c;
  arma::vec b_c;
  bool enable_constraints;

  int dmp_mod; // 1:admittance, 2:controller

  // Training data for the DMP
  bool start_train_flag; // if true logTrainData() can start
  arma::vec p, p_prev, dp, dp_prev, ddp; // store current and previous positions and velocities for numerical diff
  double a_filt; // Coefficient of LP filter: y_dot = (1-a_filt)*y_dot + a_filt*(y-y_prev)/dt

  // ========= Controller params ===========
  arma::mat M; ///< impedance controller inertia matrix
  arma::mat D; ///< impedance controller damping matrix
  arma::mat M_d; ///< model reference inertia
  double k_click; // CLICK

  arma::vec Y0; // initial position for execution
  arma::vec Y_ref, dY_ref, ddY_ref; // produced by the DMP
  arma::vec Y_c; // DMP velocity coupling
  arma::vec Z_c; // DMP acceleration coupling
  double a_force; // iir 1st order filter coeff for filtering force measurements

  // starting conditions
  bool start_exec_flag; // if true run() can start
  double f_thres; // if norm(f_ext) > f_thres then the controller can start running

  geometry_msgs::Vector3 force_msg;
  ros::Publisher force_pub;
  ros::NodeHandle ros_node;
};

#endif // DMP_EKF_CONTROLLER_H
