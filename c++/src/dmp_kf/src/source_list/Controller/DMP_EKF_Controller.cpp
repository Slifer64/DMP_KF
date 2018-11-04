#include <dmp_kf/Controller/DMP_EKF_Controller.h>
#include <dmp_kf/utils.h>
#include <ros/package.h>
#include <io_lib/parser.h>

DMP_EKF_Controller::DMP_EKF_Controller(std::shared_ptr<Robot> &robot, std::shared_ptr<GUI> &gui):
Controller(robot, gui)
{
  readControllerParams();
  readTrainingParams();

  can_clock_ptr.reset(new as64_::CanonicalClock(1.0));
  shape_attr_gating_ptr.reset(new as64_::SigmoidGatingFunction(1.0, 0.97));

  robot->update();
  q_start = robot->getJointPosition();

  force_pub = ros_node.advertise<geometry_msgs::Vector3>("f_ext", 1);

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

void DMP_EKF_Controller::readControllerParams(const char *params_file)
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
  if (!parser.getParam("a_p", a_p)) a_p = 1.0;
  if (!parser.getParam("enable_constraints", enable_constraints)) enable_constraints = false;
  arma::vec g_up_bound, g_low_bound;
  double tau_up_bound, tau_low_bound;
  if (!parser.getParam("g_up_bound", g_up_bound)) g_up_bound = arma::vec({1.0, 1.0, 1.0});
  if (!parser.getParam("g_low_bound", g_low_bound)) g_low_bound = -arma::vec({1.0, 1.0, 1.0});
  if (!parser.getParam("tau_up_bound", tau_up_bound)) tau_up_bound = 1000;
  if (!parser.getParam("tau_low_bound", tau_low_bound)) tau_low_bound = 0.1;
  arma::vec theta_up_bound = arma::join_vert(g_up_bound, arma::vec({tau_up_bound}));
  arma::vec theta_low_bound = arma::join_vert(g_low_bound, arma::vec({tau_low_bound}));
  int N_params = theta_up_bound.size();
  A_c = arma::join_vert(-arma::mat().eye(N_params, N_params), arma::mat().eye(N_params, N_params));
  b_c = arma::join_vert(-theta_low_bound, theta_up_bound);

  if (!parser.getParam("g_scale", g_scale)) g_scale = arma::vec({1.0, 1.0, 1.0});
  if (!parser.getParam("tau_scale", tau_scale)) tau_scale = 1.0;

  // starting conditions for controller run
  if (!parser.getParam("f_thres", f_thres)) f_thres = 1.5;

  // target impedance params
  if (!parser.getParam("M", M)) M = arma::vec({1.0, 1.0, 1.0});
  if (!parser.getParam("D", D)) D = arma::vec({40.0, 40.0, 40.0});

  if (!parser.getParam("M_d", M_d)) M_d = arma::vec({15.0, 15.0, 15.0});

  if (!parser.getParam("k_click", k_click)) k_click = 0.0;
  if (!parser.getParam("a_force", a_force)) a_force = 1.0;

  if (!parser.getParam("dmp_mod", dmp_mod)) dmp_mod = 1.0;
}

void DMP_EKF_Controller::initVariables()
{
  this->robot->init();

  arma::vec p = this->robot->getTaskPosition();

  start_exec_flag = false;

  // controller variables
  Y = p;
  dY.zeros(3);
  ddY.zeros(3);
  f_ext.zeros(3);
  f_ext_raw.zeros(3);

  Y_ref = Y;
  dY_ref.zeros(3);
  ddY_ref.zeros(3);

  Y_c.zeros(3);
  Z_c.zeros(3);

  // model variables
  g_d = g_d;
  tau_d = tau_d;

  Y0 = p;

  t = 0.0;
  can_clock_ptr->setTau(tau_d);
}

void DMP_EKF_Controller::initEKF()
{
  arma::vec g_hat = g_d;
  double tau_hat = tau_d;
  arma::vec theta = arma::join_vert(g_hat, arma::mat({tau_hat}));
  arma::mat P_theta = arma::diagmat( arma::join_vert( P0_g_hat, arma::mat({P0_tau_hat}) ) );

  int N_msr = dmp.size();

  ekf.reset(new as64_::kf_::EKF(theta, P_theta, N_msr, &DMP_EKF_Controller::stateTransFun, this, &DMP_EKF_Controller::msrFun, this));
  ekf->setFadingMemoryCoeff(a_p);
  ekf->enableParamsContraints(enable_constraints);
  ekf->setParamsConstraints(A_c, b_c);
  ekf->setProcessNoiseCov(Q_w);
  ekf->setMeasureNoiseCov(R_v);
  ekf->setStateTransFunJacob(&DMP_EKF_Controller::stateTransFunJacob, this);
  ekf->setMsrFunJacob(&DMP_EKF_Controller::msrFunJacob, this);
}

void DMP_EKF_Controller::initExecution()
{
  readControllerParams();

  if (gui->logControllerData()) exec_data.clear();

  initVariables();

  initEKF();
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

  dY_ref = dY;
  Y_ref = Y;

  if (gui->logControllerData()) exec_data.log(t, Y, dY, ddY, Y_ref, dY_ref, ddY_ref, f_ext_raw, f_ext, ekf->theta, ekf->P);

  // this->robot->update();

  // ========  leader-follower role  ========
  f_ext_raw = (robot->getTaskWrench()).subvec(0,2);
  f_ext = (1-a_force)*f_ext + a_force*f_ext_raw;

  ddY_ref = msrFun(ekf->theta);

  // ========  Controller  ========
  if (dmp_mod == 1) ddY = ( - D%dY + f_ext) / M;
  else if (dmp_mod == 2) ddY = ddY_ref + f_ext/M_d;
  else ddY.zeros();

  arma::vec Y_robot = this->robot->getTaskPosition();
  arma::vec V_cmd = arma::vec().zeros(6);
  V_cmd.subvec(0,2) = dY + k_click*(Y-Y_robot);
  robot->setTaskVelocity(V_cmd);

  // ========  KF update  ========
  ekf->predict(); // time update
  ekf->correct(ddY); // measurement update

  // ========  numerical integration  ========
  double Ts = robot->getControlCycle();
  t = t + Ts;
  Y = Y + dY*Ts;
  dY = dY + ddY*Ts;
}

bool DMP_EKF_Controller::simulate()
{
  // ========  Read parameters  ========
  readControllerParams();
  initVariables();
  initEKF();

  // ========  Initialization  ========
  this->robot->update();
  arma::vec p = this->robot->getTaskPosition();

  t = 0.0;
  Y = p;
  dY.zeros(3);
  ddY.zeros(3);
  Y_c.zeros(3);
  Z_c.zeros(3);
  Y0 = p;

  arma::vec Yg = g_d%g_scale;
  double tau = tau_d*tau_scale;
  arma::vec theta = arma::join_vert(Yg, arma::vec({tau}));

  can_clock_ptr->setTau(tau);

  f_ext = f_ext_raw = arma::vec().zeros(3);

  if (gui->logSimulationData()) sim_data.clear();

  // ========  Simulation loop  ========
  while (true)
  {
    if (gui->getState() == Ui::ProgramState::STOP_PROGRAM)
    {
      setErrMsg("Controller simulation was interrupted!");
      return false;
    }

    // ========  Robot update  ========
    if (this->robot->isOk() == false)
    {
      setErrMsg(robot->getErrMsg() + "\nStopping execution.");
      return false;
    }
    this->robot->update();

    // ========  Robot command  ========
    arma::vec Y_robot = this->robot->getTaskPosition();
    arma::vec V_cmd = arma::vec().zeros(6);
    V_cmd.subvec(0,2) = dY + k_click*(Y-Y_robot);
    robot->setTaskVelocity(V_cmd);
    robot->command();

    // ========  Data logging  ========
    if (gui->logSimulationData()) sim_data.log(t, Y, dY, ddY, Y_ref, dY_ref, ddY_ref, f_ext_raw, f_ext, ekf->theta, ekf->P);

    // ========  Calculate actual and estimated output (acceleration)  ========
    ddY = msrFun(theta);
    arma::vec ddY_hat =msrFun(ekf->theta);

    // estimate external force
    f_ext = f_ext_raw = (ddY - ddY_hat);

    // ========  KF update  ========
    ekf->predict(); // time update
    ekf->correct(ddY); // measurement update

    // ========  numerical integration  ========
    double Ts = robot->getControlCycle();

    t = t + Ts;
    Y = Y + dY*Ts;
    dY = dY + ddY*Ts;

    // ========  stopping criteria ========
    double err = arma::norm(Y-Yg);
    // if (err < 0.5e-3) break;
    if (t >= tau) break;
  }

  // std::cout << "###### ekf->theta = " << ekf->theta.t() << "\n";
  // std::cout << "###### theta = " << theta.t() << "\n";

  return true;

}

void DMP_EKF_Controller::initDemo()
{
  t = 0;
  p = p_prev = robot->getTaskPosition();
  dp = dp_prev = arma::vec().zeros(3);
  ddp = arma::vec().zeros(3);

  train_data.clear();
  train_data.log(t, p, dp, ddp);

  setStartPose();
}

void DMP_EKF_Controller::logDemoData()
{
  double Ts = robot->getControlCycle();
  t = t + Ts;

  p_prev = p;
  p = robot->getTaskPosition();

  dp_prev = dp;
  // dp = (1-a_filt)*dp + a_filt*(p - p_prev)/Ts;
  dp = (p - p_prev)/Ts;

  // ddp = (1-a_filt)*ddp + a_filt*(dp - dp_prev)/Ts;
  ddp = (dp - dp_prev)/Ts;

  train_data.log(t, p, dp, ddp);
}

bool DMP_EKF_Controller::train(std::string &err_msg)
{
  dmp.clear();
  readTrainingParams();

  start_train_flag = false;

  if (train_data.isempty())
  {
    err_msg = "Error training model: The training data are empty...";
    return false;
  }

  double v_start_thres = 0.002;
  double v_end_thres = 0.005;
  train_data.trim(v_start_thres, v_end_thres);

  int dim = train_data.Y_data.n_rows;
  dmp.resize(dim);
  arma::vec train_err(dim);
  for (int i=0; i<dim; i++)
  {
    dmp[i].reset(new as64_::DMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr));
    train_err(i) = dmp[i]->train(train_method, train_data.Time, train_data.Y_data.row(i), train_data.dY_data.row(i), train_data.ddY_data.row(i), true);
  }
  std::ostringstream out;
  out << "Training error:\n" << train_err;
  gui->printMsg(out.str().c_str(), Ui::MSG_TYPE::INFO);
  // std::cout << "train_err = \n" << train_err << "\n";


  is_trained = true;

  g_d = train_data.getFinalPoint();
  tau_d = train_data.getTimeDuration();

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

bool DMP_EKF_Controller::runModel()
{
  // ========  Read parameters  ========
  readControllerParams();

  this->robot->update();
  arma::vec p = this->robot->getTaskPosition();

  Y = p;
  dY.zeros(3);
  ddY.zeros(3);
  Y0 = p;

  arma::vec Yg = g_d;
  double tau = tau_d;
  double t = 0.0;
  can_clock_ptr->setTau(tau);

  if (gui->logModelRunData()) modelRun_data.clear();

  arma::vec q = robot->getJointPosition();

  // ========  model run loop  ========
  while (true)
  {
    if (gui->getState() == Ui::ProgramState::STOP_PROGRAM)
    {
      setErrMsg("Model run execution was interrupted!");
      return false;
    }

    // ========  robot update  ========
    if (this->robot->isOk() == false)
    {
      setErrMsg(robot->getErrMsg() + "\nStopping execution.");
      return false;
    }
    this->robot->update();

    // ========  robot command  ========
    arma::vec Y_robot = this->robot->getTaskPosition();
    arma::vec V_cmd = arma::vec().zeros(6);
    V_cmd.subvec(0,2) = dY + k_click*(Y-Y_robot);

    robot->setTaskVelocity(V_cmd);
    robot->command();

    // ========  data logging  ========
    if (gui->logModelRunData()) modelRun_data.log(t, Y, dY, ddY);

    int dim = dmp.size();
    for (int i=0; i<dim; i++)
    {
      double y_c=0, z_c=0;
      ddY(i) = dmp[i]->getAccel(Y(i), dY(i), Y0(i), y_c, z_c, t/tau, Yg(i), tau);
    }

    // ========  numerical integration  ========
    double Ts = robot->getControlCycle();

    t = t + Ts;
    Y = Y + dY*Ts;
    dY = dY + ddY*Ts;

    // ========  stopping criteria  ========
    double err = arma::norm(Y-Yg);
    if (t>=tau && err<0.5e-3) break;
  }

  return true;

}

arma::vec DMP_EKF_Controller::msrFun(const arma::vec &theta, void *cookie)
{
  int i_end = theta.size()-1;
  arma::vec Yg_hat = theta.subvec(0, i_end-1);
  double tau_hat = theta(i_end);
  double x_hat = t/tau_hat;

  // ========  KF estimation  ========
  int dim = dmp.size();
  arma::vec Z(dim);
  for (int i=0; i<dim; i++)
  {
    Z(i) = dmp[i]->getAccel(Y(i), dY(i), Y0(i), Y_c(i), Z_c(i), x_hat, Yg_hat(i), tau_hat);
  }

  return Z;
}

arma::vec DMP_EKF_Controller::stateTransFun(const arma::vec &theta, void *cookie)
{
  return theta;
}

arma::mat DMP_EKF_Controller::msrFunJacob(const arma::vec &theta, void *cookie)
{
  int i_end = theta.size()-1;
  arma::vec Yg_hat = theta.subvec(0, i_end-1);
  double tau_hat = theta(i_end);
  double x_hat = t/tau_hat;

  // ========  KF estimation  ========
  int dim = dmp.size();
  int n_theta = theta.size();
  arma::mat J = arma::mat().zeros(dim, n_theta);
  for (int i=0; i<dim; i++)
  {
    arma::vec J_i = dmp[i]->getAcellPartDev_g_tau(t, Y(i), dY(i), Y0(i), x_hat, Yg_hat(i), tau_hat);
    J(i,i) = J_i(0);
    J(i,n_theta-1) = J_i(1);
  }
  return J;
}

arma::mat DMP_EKF_Controller::stateTransFunJacob(const arma::vec &theta, void *cookie)
{
  int N = theta.size();
  return arma::mat().eye(N,N);
}
