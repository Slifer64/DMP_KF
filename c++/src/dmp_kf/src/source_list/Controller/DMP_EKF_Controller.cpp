#include <dmp_kf/Controller/DMP_EKF_Controller.h>
#include <dmp_kf/utils.h>
#include <ros/package.h>
#include <io_lib/parser.h>

DMP_EKF_Controller::DMP_EKF_Controller(std::shared_ptr<Robot> &robot):Controller(robot)
{
  can_clock_ptr.reset(new as64_::CanonicalClock(1.0));
  shape_attr_gating_ptr.reset(new as64_::SigmoidGatingFunction(1.0, 0.95));

  robot->update();
  q_start = robot->getJointPosition();
}

void DMP_EKF_Controller::readTrainingParams(const char *params_file)
{
  std::string path_to_config_file;
  if (params_file != NULL) path_to_config_file = *params_file;
  else path_to_config_file = ros::package::getPath(PACKAGE_NAME)+ "/config/Controller_config.yml";
  as64_::io_::Parser parser(path_to_config_file);

  // train data params
  if (!parser.getParam("N_kernels", N_kernels)) N_kernels = 20;
  if (!parser.getParam("train_method", train_method)) train_method = "LWR";
  if (!parser.getParam("a_z", a_z)) a_z = 20;
  if (!parser.getParam("b_z", b_z)) b_z = a_z/4;
  if (!parser.getParam("a_filt", a_filt)) a_filt = 0.5;
}

void DMP_EKF_Controller::readExecutionParams(const char *params_file)
{
  std::string path_to_config_file;
  if (params_file != NULL) path_to_config_file = *params_file;
  else path_to_config_file = ros::package::getPath(PACKAGE_NAME)+ "/config/Controller_config.yml";
  as64_::io_::Parser parser(path_to_config_file);

  if (!parser.getParam("P0_g_hat", P0_g_hat)) P0_g_hat = arma::vec({1.0, 1.0, 1.0});
  if (!parser.getParam("P0_tau_hat", P0_tau_hat)) P0_tau_hat = 1.0;
  if (!parser.getParam("R_v", R_v)) R_v = arma::vec({1.0, 1.0, 1.0});
  R_v = arma::diagmat(R_v);
  inv_R_v = arma::inv(R_v);
  if (!parser.getParam("Q_W", Q_w)) Q_w = arma::vec({0.0, 0.0, 0.0});
  Q_w = arma::diagmat(Q_w);
  if (!parser.getParam("a_p", a_p)) a_p = 0.0;

  // starting conditions for controller run
  if (!parser.getParam("f_thres", f_thres)) f_thres = 1.5;

  // target impedance params
  if (!parser.getParam("M", M)) M = arma::vec({1.0, 1.0, 1.0});
  if (!parser.getParam("K", K)) K = arma::vec({200.0, 200.0, 200.0});
  if (!parser.getParam("D", D)) D = 2*arma::sqrt(M%K);
  if (!parser.getParam("k_click", k_click)) k_click = 0.0;
  if (!parser.getParam("ff_gains", ff_gains)) ff_gains = arma::vec({1.0, 1.0, 1.0}); // feedforward gains for the target impedance model
  if (!parser.getParam("a_force", a_force)) a_force = 1.0;

  // leader-follower sigmoid function params  1 / ( 1 + exp(a_m*(norm(F)-c_m)) )
  if (!parser.getParam("a_m", a_m)) a_m = 3.5;
  if (!parser.getParam("c_m", c_m)) c_m = 1.3;
}

void DMP_EKF_Controller::initExecution()
{
  readExecutionParams();

  this->robot->update();
  arma::vec p = this->robot->getTaskPosition();

  start_exec_flag = false;

  // controller variables
  Y = p;
  dY.zeros(3);
  ddY.zeros(3);
  U_dmp.zeros(3);
  U_total.zeros(3);
  f_ext.zeros(3);

  Y_ref.zeros(3);
  dY_ref.zeros(3);
  ddY_ref.zeros(3);

  // model variables
  int i_end = Timed.size()-1;
  g_d = Yd_data.col(i_end);
  tau_d = Timed(i_end);

  Y0 = p;
  g_hat = g_d;
  tau_hat = tau_d;
  theta = arma::join_vert(g_hat, arma::mat({tau_hat}));
  P_theta = arma::diagmat( arma::join_vert( P0_g_hat, arma::mat({P0_tau_hat}) ) );
  t = 0.0;
  x_hat = t/tau_hat;
  mf = 1.0;
  can_clock_ptr->setTau(tau_hat);
}

bool DMP_EKF_Controller::startExecution()
{
  if (!start_exec_flag)
  {
    arma::vec F_ext = robot->getTaskWrench();
    if (arma::norm(F_ext) > f_thres) start_exec_flag = true;
    else start_exec_flag = false;
  }
  return start_exec_flag;
}

void DMP_EKF_Controller::execute()
{
  // ========  Initial starting conditions  ========
  if (!startExecution())
  {
    robot->setTaskVelocity({0,0,0,0,0,0});
    return;
  }

  this->robot->update();

  // ========  leader-follower role  ========
  arma::vec f_ext_new = (robot->getTaskWrench()).subvec(0,2);
  f_ext = (1-a_force)*f_ext + a_force*f_ext_new;
  mf = 1 / ( 1 + std::exp( a_m*(arma::norm(f_ext)-c_m) ) );

  // ========  KF estimation  ========
  int dim = dmp.size();
  int n_theta = theta.size();
  arma::mat dC_dtheta = arma::mat().zeros(dim, n_theta);
  for (int i=0; i<dim; i++)
  {
    double y_c=0, z_c=0;
    ddY_ref(i) = dmp[i]->getAccel(Y(i), dY(i), Y0(i), y_c, z_c, x_hat, g_hat(i), tau_hat);
    arma::vec dC_dtheta_i = dmp[i]->getAcellPartDev_g_tau(t, Y(i), dY(i), Y0(i), x_hat, g_hat(i), tau_hat);
    dC_dtheta(i,i) = dC_dtheta_i(0);
    dC_dtheta(i,n_theta-1) = dC_dtheta_i(1);
  }
  dY_ref = dY;
  Y_ref = Y;

  // ========  Controller  ========
  U_dmp = -K%(Y - Y_ref) + D%dY_ref + M%ddY_ref;

  U_total = mf*U_dmp + (1-mf)*(ff_gains%f_ext);

  ddY = ( - D%dY - K%Y + U_total) / M;

  arma::vec Y_robot = this->robot->getTaskPosition();
  arma::vec V_cmd = arma::vec().zeros(6);
  V_cmd.subvec(0,2) = dY + k_click*(Y-Y_robot);
  robot->setTaskVelocity(V_cmd);

  // ========  KF update  ========
  arma::mat K_kf = P_theta*dC_dtheta.t()*inv_R_v;
  arma::vec theta_dot = K_kf * (ddY - ddY_ref);
  arma::mat P_dot = Q_w - K_kf*dC_dtheta*P_theta + 2*a_p*P_theta;

  // ========  numerical integration  ========
  double Ts = robot->getControlCycle();

  t = t + Ts;
  Y = Y + dY*Ts;
  dY = dY + ddY*Ts;

  theta = theta + theta_dot*Ts;
  g_hat = theta.subvec(0,2);
  tau_hat = theta(3);
  P_theta = P_theta + P_dot*Ts;
  x_hat = t/tau_hat;
}

void DMP_EKF_Controller::logExecData()
{

}

void DMP_EKF_Controller::initDemo()
{
  readTrainingParams();

  Timed.clear();
  Yd_data.clear();
  dYd_data.clear();
  ddYd_data.clear();
  dmp.clear();

  t_d = 0;
  p = p_prev = robot->getTaskPosition();
  dp = dp_prev = arma::vec().zeros(3);
  ddp = arma::vec().zeros(3);

  Timed = arma::mat({0});
  Yd_data = p;
  dYd_data = dp;
  ddYd_data = ddp;

  q_start = robot->getJointPosition();
  is_q_start_set = true;
}

void DMP_EKF_Controller::logDemoData()
{
  this->robot->update();

  double Ts = robot->getControlCycle();
  t_d = t_d + Ts;

  p_prev = p;
  p = robot->getTaskPosition();

  dp_prev = dp;
  dp = (1-a_filt)*dp + a_filt*(p - p_prev)/Ts;

  ddp = (1-a_filt)*ddp + a_filt*(dp - dp_prev)/Ts;

  Timed = arma::join_horiz(Timed, arma::mat({t_d}));
  Yd_data = arma::join_horiz(Yd_data, p);
  dYd_data = arma::join_horiz(dYd_data, dp);
  ddYd_data = arma::join_horiz(ddYd_data, ddp);
}

void DMP_EKF_Controller::clearDemoData()
{
  Timed.clear();
  Yd_data.clear();
  dYd_data.clear();
  ddYd_data.clear();
}

bool DMP_EKF_Controller::train(std::string &err_msg)
{
  start_train_flag = false;

  if (Timed.size() == 0)
  {
    err_msg = "Error training model: The training data are empty...";
    return false;
  }

  int dim = Yd_data.n_rows;
  dmp.resize(dim);
  for (int i=0; i<dim; i++)
  {
    dmp[i].reset(new as64_::DMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr));
    dmp[i]->train(train_method, Timed, Yd_data.row(i), dYd_data.row(i), ddYd_data.row(i));
  }
  is_trained = true;

  int n_data = Yd_data.n_cols;
  g_d = Yd_data.col(n_data-1);
  tau_d = Timed(n_data-1);

  return true;
}

bool DMP_EKF_Controller::saveTrainedModel(std::string &err_msg)
{
  if (!is_trained)
  {
    err_msg = "Error saving the model:\nThe model hasn't been trained...";
    return false;
  }

  std::string data_file = ros::package::getPath(PACKAGE_NAME)+ "/data/model_data.bin";
  bool binary = true;

  std::ofstream out(data_file.c_str(), std::ios::binary);
  if (!out)
  {
    err_msg = "Error saving the model:\nCouldn't create file \"" + data_file + "\"...";
    return false;
  }

  write_mat(q_start, out, binary);
  write_mat(g_d, out, binary);
  write_scalar((double)tau_d, out, binary);

  int dim = dmp.size();
  write_scalar((long)dim, out, binary);
  for (int i=0; i<dim; i++)
  {
    arma::vec dmp_params(3);
    dmp_params(0) = dmp[i]->N_kernels;
    dmp_params(1) = dmp[i]->a_z;
    dmp_params(2) = dmp[i]->b_z;
    dmp_params = arma::join_vert(dmp_params, dmp[i]->w);
    write_mat(dmp_params, out, binary);
  }

  out.close();

  return true;
}

bool DMP_EKF_Controller::loadTrainedModel(std::string &err_msg)
{
  std::string data_file = ros::package::getPath(PACKAGE_NAME)+ "/data/model_data.bin";
  bool binary = true;

  std::ifstream in(data_file.c_str(), std::ios::binary);
  if (!in)
  {
    err_msg = "Error loading the model:\nCouldn't open file: \"" + data_file + "\"";
    return false;
  }

  read_mat(q_start, in, binary);
  read_mat(g_d, in, binary);
  read_scalar(tau_d, in, binary);

  long dim;
  read_scalar(dim, in, binary);
  dmp.resize(dim);
  for (int i=0; i<dim; i++)
  {
    arma::vec dmp_params;
    read_mat(dmp_params, in, binary);
    int i_end = dmp_params.size()-1;
    int N_kernels = dmp_params(0);
    double a_z = dmp_params(1);
    double b_z = dmp_params(2);

    dmp[i].reset(new as64_::DMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr));
    dmp[i]->w = dmp_params.subvec(3,i_end);
  }

  in.close();

  is_trained = true;

  return true;
}

void DMP_EKF_Controller::runModel()
{
  initExecution();

  arma::rowvec Time_data;
  arma::mat Y_data;
  arma::mat dY_data;
  arma::mat ddY_data;

  while (true)
  {
    if (this->robot->isOk() == false) break;
    this->robot->update();

    Time_data = arma::join_horiz(Time_data, arma::mat({t}));
    Y_data = arma::join_horiz(Y_data, Y);
    dY_data = arma::join_horiz(dY_data, dY);
    ddY_data = arma::join_horiz(ddY_data, ddY);

    int dim = dmp.size();
    for (int i=0; i<dim; i++)
    {
      double y_c=0, z_c=0;
      ddY(i) = dmp[i]->getAccel(Y(i), dY(i), Y0(i), y_c, z_c, x_hat, g_hat(i), tau_hat);
    }

    arma::vec Y_robot = this->robot->getTaskPosition();
    arma::vec V_cmd = arma::vec().zeros(6);
    V_cmd.subvec(0,2) = dY + k_click*(Y-Y_robot);
    robot->setTaskVelocity(V_cmd);
    robot->command();

    // ========  numerical integration  ========
    double Ts = robot->getControlCycle();

    t = t + Ts;
    Y = Y + dY*Ts;
    dY = dY + ddY*Ts;
    x_hat = t/tau_hat;

    double err = arma::norm(Y-g_hat);
    if (err < 0.5e-3) break;
  }

  std::string data_file = ros::package::getPath(PACKAGE_NAME)+ "/data/model_run_data.bin";
  bool binary = true;

  std::ofstream out(data_file.c_str(), std::ios::binary);
  if (!out)
  {
    throw std::ios_base::failure(std::string("Error saving training model data:\nCouldn't create file: \"" + data_file + "\""));
  }

  write_mat(Time_data, out, binary);
  write_mat(Y_data, out, binary);
  write_mat(dY_data, out, binary);
  write_mat(ddY_data, out, binary);

  out.close();

}
