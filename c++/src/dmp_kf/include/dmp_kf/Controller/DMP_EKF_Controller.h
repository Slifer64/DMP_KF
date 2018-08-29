#ifndef DMP_EKF_CONTROLLER_H
#define DMP_EKF_CONTROLLER_H

#include <dmp_lib/dmp_lib.h>

#include <dmp_kf/DMP_KF/DMP_KF.h>

#include <dmp_kf/utils.h>

#include <ros/ros.h>

class DMP_EKF_Controller: public Controller
{
public:
  DMP_EKF(std::shared_ptr<Robot> &robot);
  ~DMP_EKF() {}

  void init();
  bool start();
  void run();

  void initTraining();
  void logTrainData();
  void train();

private:
  void readParams(const char *params_file = NULL);
  void initParams();
  void initVars();

  arma::vec Y0, Y, dY, ddY;
  arma::vec Y_ref, dY_ref, ddY_ref;
  arma::vec f_ext;

  // starting conditions
  bool start_flag;
  double f_thres;

  // leader-follower sigmoid function params
  double a_m;
  double c_m;

  // model
  arma::vec<as64_::DMP> dmp;
  arma::vec g_hat; // goal estimate
  double tau_hat; // time scale estimate
  arma::mat P_theta; // covariance of g_hat and tau_hat
  arma::mat R_v, inv_R_v;
  double a_p;
  arma::mat Q_w;

  // Training data for the DMP
  std::string train_method;
  double t;
  arma::rowvec Timed;
  arma::mat Yd, dYd, ddYd;
  arma::vec p, p_prev, dp, dp_prev;
  double a_filt;

  // ========= Controller params ===========
  arma::mat M; ///< impedance controller inertia matrix
  arma::mat K; ///< impedance controller stiffness matrix
  arma::mat D; ///< impedance controller damping matrix
};

#endif // DMP_EKF_CONTROLLER_H
