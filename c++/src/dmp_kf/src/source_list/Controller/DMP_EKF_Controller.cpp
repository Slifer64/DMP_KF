#include <dmp_kf/Controller/DMP_EKF_Controller.h>
#include <param_lib/param_lib.h>
#include <math_lib/math_lib.h>

DMP_EKF_Controller::DMP_EKF_Controller(std::shared_ptr<Robot> &robot):Controller(robot)
{
  S.zeros(N_DOFS);
  dS.zeros(N_DOFS);
  ddS.zeros(N_DOFS);
}

void DMP_EKF_Controller::readParams(const char *params_file)
{
  std::string path_to_config_file;
  if (params_file != NULL) path_to_config_file = *params_file;
  else path_to_config_file = ros::package::getPath(PACKAGE_NAME)+ "/config/Controller_config.yml";
  as64_::param_::Parser parser(path_to_config_file);

  // arma::vec M_vec;
  // if (!parser.getParam("M", M_vec)) M_vec = arma::vec().ones(N_DOFS);
  // M = arma::diagmat(M_vec);
  // arma::vec K_vec;
  // if (!parser.getParam("K", K_vec)) K_vec = arma::vec().ones(N_DOFS)*100;
  // K = arma::diagmat(K_vec);
  // arma::vec D_vec;
  // if (!parser.getParam("D", D_vec)) D_vec = arma::sqrt(2*M_vec%K_vec);
  // D = arma::diagmat(D_vec);
  //
  // if (!parser.getParam("Fext_dead_zone", Fext_dead_zone)) Fext_dead_zone = arma::vec().zeros(6);
  //
  // if (!parser.getParam("k_click", k_click)) k_click = 1.0;

}

void DMP_EKF_Controller::init()
{
  initParams();
  initVars();
}

void DMP_EKF_Controller::run()
{
  if (!start())
  {
    robot->setTaskVelocity({0,0,0,0,0,0});
    return;
  }

  f_ext = (robot->getTaskWrench()).subvec(0,2);
  double m = 1 / ( 1 + std::exp( a_m*(std::norm(f_ext)-c_m) ) );

  x_hat = t/tau_hat;
  for (int i=0; i<dmp.size(); i++)
  {
    ddY_ref(i) = dmp[i].getAccel(Y(i), dY(i), Y0(i), 0, 0, x_hat, g_hat(i), tau_hat);
  }
  dY_ref = dY;
  Y_ref = Y;

  arma::vec V = -K*(Y - Y_ref) + D*dY_ref + M*ddY_ref;

  arma::vec U = m*V + (1-m)*(ff_gains%f_ext);

  ddY = arma::pinv(M) * ( - D*dY - K*Y + U);

  robot->setTaskVelocity(dY);

  // KF update
  arma::mat dC_dtheta;
  arma::mat K_kf = P_theta*dC_dtheta.t()*inv_R_v;
  arma::vec theta_dot = K_kf * (ddY - ddY_ref);
  arma::mat P_dot = Q_w - K_kf*dC_dtheta*P_theta + 2*a_p*P_theta;

  arma::vec g_hat_dot = theta.subvec(0,2);
  double tau_hat_dot = theta(3);

  theta = theta + theta_dot*dt;


  // numerical integration

  double Ts = robot->getControlCycle();

  t = t + Ts;
  Y = Y + dY*Ts;
  dY = dY + ddY*Ts;

  g_hat = g_hat + g_hat_dot*Ts;
  tau_hat = tau_hat + tau_hat_dot*Ts;
  P_theta = P_theta + P_dot*Ts;
}

bool DMP_EKF_Controller::start()
{
  if (!start_flag)
  {
    arma::vec F_ext = robot->getTaskWrench();
    if (arma::norm(F_ext) > 0.2)
    {
      start_flag = true;
      S0 = robot->getTaskPosition();
      t = 0.0;
    }
    else start_flag = false;
  }
  return start_flag;
}

void DMP_EKF_Controller::initTraining()
{
  Timed.clear();
  Yd.clear();
  dYd.clear();
  ddYd.clear();

  t = 0;
  p = p_prev = robot->getTaskPosition();
  dp = dp_prev = arma::vec().zeros(3);

  Timed = arma::mat({0});
  Yd = p;
  dYd = dp;
  ddYd = arma::vec().zeros(3);
}

void DMP_EKF_Controller::logTrainData()
{
  double Ts = robot->getControlCycle();
  t = t + Ts;

  p_prev = p;
  p = robot->getTaskPosition();

  dp_prev = dp;
  dp = (1-a_filt)*dp + a_filt*(p - p_prev)/Ts;

  ddp = (1-a_filt)*dpp + a_filt*(dp - dp_prev)/Ts;

  Timed = arma::join_horiz(Timed, arma::mat({t}));
  Yd = arma::join_horiz(Yd, p);
  dYd = arma::join_horiz(dYd, dp);
  ddYd = arma::join_horiz(Yd, ddp);
}

void DMP_EKF_Controller::train()
{
  int N = dmp.size();

  for (int i=0; i<N; i++)
  {
    dmp[i].train(train_method, Timed, Yd.row(i), dYd.row(i), ddYd.row(i));
  }
}

void DMP_EKF_Controller::initParams()
{
  readParams();
}

void DMP_EKF_Controller::initVars()
{
  this->robot->update();
  arma::vec p = this->robot->getTaskPosition();
  arma::vec Q = this->robot->getTaskOrientation();

  start_flag = false;

  S = p;
  g_est = S(1);
  dS.zeros(N_DOFS);
  ddS.zeros(N_DOFS);
  U.zeros(N_DOFS);
  V.zeros(N_DOFS);
  F_c.zeros(N_DOFS);
  F_c_prev.zeros(N_DOFS);
  F_c_d.zeros(N_DOFS);
  F_c_d_prev.zeros(N_DOFS);
  M_o = arma::mat().eye(3,3) * m_o;
  M_o(2,2) = 0.2 * m_o;

  Fext.zeros(6);
  Fext_prev.zeros(6);
  Fext_filt.zeros(6);
  start_lifting = false;

  is_okay = true;

  S_rot.zeros(3);
  S_rot(2) = Q(3);
  dS_rot.zeros(3);
  ddS_rot.zeros(3);
}
