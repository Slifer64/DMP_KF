#include <OL_2D_rup/Controller/ImpVelController.h>
#include <param_lib/param_lib.h>
#include <math_lib/math_lib.h>
#include <sigproc_lib/sigproc_lib.h>

arma::vec barVec(const arma::vec &x)
{
  arma::vec x_bar(2);
  x_bar(0) = -x(1);
  x_bar(1) = x(0);
  return x_bar;
}

arma::mat gMat(const arma::vec &x)
{
  arma::mat Gx = arma::mat().eye(3,3);
  Gx.submat(0,2,1,2) = barVec(x);

  return Gx;
}

arma::mat rotz(double theta)
{
  arma::mat Rz = {{cos(theta), -sin(theta)}, {sin(theta), cos(theta)}};
  return Rz;
}

ImpVelController::ImpVelController(std::shared_ptr<Robot> &robot, std::shared_ptr<RefModel> &ref_model):Controller(robot,ref_model)
{
  S.zeros(N_DOFS);
  dS.zeros(N_DOFS);
  ddS.zeros(N_DOFS);
  U.zeros(N_DOFS);
  V.zeros(N_DOFS);
  F_c.zeros(N_DOFS);
  F_c_prev.zeros(N_DOFS);
  F_c_d.zeros(N_DOFS);
  F_c_d_prev.zeros(N_DOFS);
  M_o = arma::mat().eye(3,3);

  S_rot.zeros(3);
  dS_rot.zeros(3);
  ddS_rot.zeros(3);
}

void ImpVelController::readParams(const char *params_file)
{
  std::string path_to_config_file;
  if (params_file != NULL) path_to_config_file = *params_file;
  else path_to_config_file = ros::package::getPath(PACKAGE_NAME)+ "/config/Controller_config.yml";
  as64_::param_::Parser parser(path_to_config_file);

  arma::vec M_vec;
  if (!parser.getParam("M", M_vec)) M_vec = arma::vec().ones(N_DOFS);
  M = arma::diagmat(M_vec);
  arma::vec K_vec;
  if (!parser.getParam("K", K_vec)) K_vec = arma::vec().ones(N_DOFS)*100;
  K = arma::diagmat(K_vec);
  arma::vec D_vec;
  if (!parser.getParam("D", D_vec)) D_vec = arma::sqrt(2*M_vec%K_vec);
  D = arma::diagmat(D_vec);

  if (!parser.getParam("M_rot", M_vec)) M_vec = arma::vec().ones(3);
  M_rot = arma::diagmat(M_vec);
  if (!parser.getParam("K_rot", K_vec)) K_vec = arma::vec().ones(3)*100;
  K_rot = arma::diagmat(K_vec);
  if (!parser.getParam("D_rot", D_vec)) D_vec = arma::sqrt(2*M_vec%K_vec);
  D_rot = arma::diagmat(D_vec);

  if (!parser.getParam("max_m_o", max_m_o)) max_m_o = 0.0;
  if (!parser.getParam("max_torque", max_torque)) max_torque = 0.0;
  if (!parser.getParam("r_CoM", r_CoM)) r_CoM = arma::vec({0, 0});
  if (!parser.getParam("pos_tol_stop", pos_tol_stop)) pos_tol_stop = 0.01;
  if (!parser.getParam("vel_tol_stop", vel_tol_stop)) vel_tol_stop = 0.001;
  if (!parser.getParam("orient_ctrl_type", orient_ctrl_type)) orient_ctrl_type = 2;

  if (!parser.getParam("Fext_dead_zone", Fext_dead_zone)) Fext_dead_zone = arma::vec().zeros(6);

  if (!parser.getParam("k_click", k_click)) k_click = 1.0;
  if (!parser.getParam("f_iir_filt", f_iir_filt)) f_iir_filt = 1.0;

  if (!parser.getParam("vel_max", vel_max)) vel_max = arma::vec().zeros(6);
  if (!parser.getParam("F_max", F_max)) F_max = arma::vec().zeros(6);
  if (!parser.getParam("ff_gains", ff_gains)) ff_gains = arma::vec().zeros(3);

  if (!parser.getParam("assume_static_obj", assume_static_obj)) assume_static_obj = false;

  if (!parser.getParam("g_est", g_est)) g_est = 0;
  if (!parser.getParam("Kg", Kg)) Kg = 0;
  if (!parser.getParam("g_est_update_gain", g_est_update_gain)) g_est_update_gain = 0.0;

}

void ImpVelController::init()
{
  initParams();
  initVars();
}

void ImpVelController::run()
{
  if (!start())
  {
    robot->setTaskVelocity({0,0,0,0,0,0});
    return;
  }

  ref_model->calcOutput(S_ref, dS_ref, ddS_ref);

  arma::vec p = robot->getTaskPosition();
  arma::vec Q = robot->getTaskOrientation();
  arma::vec S_r = getRobotState(p,Q);

  // S = ref_model->S;
  // dS = ref_model->dS;
  // ddS = ref_model->ddS;

  // Compliant Cartesian axis
  // V = -K*S + ff_gains%(F_c - F_c_d);
  Fext_filt(2) = Fext_filt(2) - F_c_d(1);
  // V = ff_gains.subvec(0,2)%Fext_filt.subvec(0,2);

  // Control applied by the robot to track its reference
  V = K*(ref_model->S-S) + D*ref_model->dS + M*ref_model->ddS + ff_gains%(F_c - F_c_d);

  V(1) += Kg*(g_est - S(1));

  // Robot model dynamics
  ddS = arma::pinv(M) * ( - D*dS + V);

  // S(2) = ref_model->S(1);
  // dS(2) = ref_model->dS(1);
  // ddS(2) = ref_model->ddS(1);

  vel_cmd = dS; // + k_click*(S-S_r);
  Vel_cmd = arma::vec().zeros(6);
  Vel_cmd(0) = 0; //vel_cmd(0); // x
  Vel_cmd(1) = 0; //vel_cmd(1); // y
  Vel_cmd(2) = vel_cmd(1) + k_click*(S(2)-S_r(2)); // z

  // calculate the torque and rotational velocity in task frame and transform them in base frame
  // get the rotation matrix of the task relative to base frame
  arma::mat R = as64_::math_::quat2rotm(robot->getTaskOrientation());
  // transform the torque to the task frame
  arma::vec torq = R.t() * Fext_filt.subvec(3,5);
  // Keep only the torque along x-axis (where we want compliance)
  torq = torq % (arma::vec{ff_gains(2),0,0});
  if (torq(0)>0) torq(0)=0.0;
  // transform it back to the base frame
  torq = R * torq;
  // calc velocity in task frame
  ddS_rot = arma::pinv(M_rot) * ( - D_rot*dS_rot - K_rot*S_rot + torq);
  Vel_cmd.subvec(3,5) = dS_rot;

  robot->setTaskVelocity(Vel_cmd);

  arma::vec model_error = this->getError();
  double dt = robot->getControlCycle();
  ref_model->update(dt, model_error);
  this->update(dt);
}

arma::vec ImpVelController::getError()
{
  calc_Fc();
  calc_Fc_d();

  arma::vec err = F_c - F_c_d;

  return err;
}

bool ImpVelController::start()
{
  if (!start_flag)
  {
    arma::vec Ferr = getError();
    if (Ferr(1) > 0.2) start_flag = true;
    else start_flag = false;
  }
  return start_flag;
}

void ImpVelController::update(double dt)
{
  arma::vec Fext = robot->getTaskWrench();
  Fext_filt = f_iir_filt*Fext + (1-f_iir_filt)*Fext_filt;

  S = S + dS*dt;
  dS = dS + ddS*dt;

  S_rot = S_rot + dS_rot*dt;
  dS_rot = dS_rot + ddS_rot*dt;
}

void ImpVelController::estimateObjectDynamics()
{
  double z_tol_stop = 0.0035;
  double duration = 0.2;
  double v_z = z_tol_stop / duration;
  double toqrue_limit = max_torque;
  double force_limit = max_m_o*9.81;

  arma::vec p, p0;
  arma::vec Fext, Fext_prev, Fext_mean;
  int count = 0;
  double a_f = 0.25;

  Robot::Mode prev_mode = robot->getMode();
  robot->setMode(Robot::Mode::VELOCITY_CONTROL);

  robot->update();
  p0 = robot->getTaskPosition();
  Fext = robot->getTaskWrench();
  Fext_prev = Fext;

  arma::vec Fext2 = Fext;
  // std::vector<as64_::spl_::MovingAverageFilter> ma_filter;
  // const int N_samples = 20;
  // for (int i=0;i<6;i++) ma_filter[i].init(N_samples, Fext2(i));

  while (robot->isOk())
  {
    count++;
    robot->setTaskVelocity({0,0,v_z,0,0,0});
    robot->command();
    robot->update();

    p = robot->getTaskPosition();
    arma::vec Fext_msr = robot->getTaskWrench();

    Fext = a_f*Fext_msr + (1-a_f)*Fext_prev;
    Fext_prev = Fext;

    // for (int i=0;i<6;i++) Fext2(i) = ma_filter[i].filter(Fext_msr(i));
    // Fext = Fext2;

    if ( -Fext(2) > force_limit )
    {
      std::cout << "*** Force limit reached ***\n";
      std::cout << "force_limit = " << force_limit << "\n";
      std::cout << "Fext = " << -Fext(2) << "\n";
      break;
    }
    else if ( arma::norm(Fext.subvec(3,5)) > toqrue_limit )
    {
      std::cout << "*** Torque limit reached ***\n";
      std::cout << "toqrue_limit = " << toqrue_limit << "\n";
      std::cout << "Torque = " <<   arma::norm(Fext.subvec(3,5)) << "\n";
      break;
    }
    else if ( (p(2)-p0(2)) > z_tol_stop )
    {
      std::cout << "*** Position limit reached ***\n";
      std::cout << "z_tol_stop = " << z_tol_stop << "\n";
      std::cout << "z = " << p(2)-p0(2) << "\n";
      break;
    }
  }

  robot->setTaskVelocity({0,0,0,0,0,0});
  robot->update();

  // arma::vec p_CoM = - arma::inv(as64_::math_::vec2ssMat(Fext.subvec(0,2))) * Fext.subvec(3,5);

  double w_est = -Fext(2)/9.81 + 0.00;
  if (w_est < 0) w_est = 0;
  m_o = (w_est > max_m_o)?max_m_o:w_est;

  F_neg_thres = (w_est - m_o)*9.81;


  std::cout << "count = " << count << "\n";
  std::cout << "w_est = " << w_est << "\n";
  std::cout << "m_o = " << m_o << "\n";
  // std::cout << "CoM: " << p_CoM.t() << "\n";

  F_c = {0, -m_o*9.81, 0};
  F_c_prev = F_c;
  F_c_d = F_c;
  F_c_d_prev = F_c_d;
}

void ImpVelController::initParams()
{
  readParams();
}

void ImpVelController::initVars()
{
  estimateObjectDynamics();

  this->robot->update();
  arma::vec p = this->robot->getTaskPosition();
  arma::vec Q = this->robot->getTaskOrientation();
  arma::vec S0 = getRobotState(p, Q);

  start_flag = false;

  S = S0;
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

  ref_model->init(S0);
}

arma::vec ImpVelController::calc_Fc_d()
{
  //m_o = M_o(0,0);
  bool is_stiff = true;

  arma::vec w_o = arma::vec({0, -m_o*grav, 0});

  double dtheta_o = dS(N_DOFS-1);
  arma::vec dp_c(N_DOFS);
  dp_c.subvec(0,1) = -r_CoM*std::pow(dtheta_o,2);
  dp_c(2) = 0.0;

  arma::mat A(6,6);
  A.submat(0,0,2,2) = M*gMat(r_CoM);
  A.submat(0,3,2,5) = -(!is_stiff)*arma::mat().eye(3,3);
  A.submat(3,0,5,2) = M_o;
  A.submat(3,3,5,5) = gMat(r_CoM).t();

  arma::vec b(6);
  b.subvec(0,2) = -D*dS + U - M*dp_c;
  b.subvec(3,5) = w_o;

  arma::vec X = arma::pinv(A)*b;

  F_c_d_prev = F_c_d;
  F_c_d = (assume_static_obj)? w_o:X.subvec(3,5);
  F_c_d = f_iir_filt*F_c_d + (1-f_iir_filt)*F_c_d_prev;

  // ddS = X.subvec(0,2);
  return F_c_d;
}

arma::vec ImpVelController::calc_Fc()
{
  // arma::vec Fext = robot->getTaskWrench();
  // arma::vec sign_Fext = arma::sign(Fext);
  // arma::vec Fext2 = Fext - sign_Fext%Fext_dead_zone;
  // Fext2 = 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);
  arma::vec Fext2 = robot->getTaskWrench();

  // if (Fext2(2) < 0) Fext2(2) += F_neg_thres;

  F_c_prev = F_c;

  F_c(0) = Fext2(0);
  F_c(1) = Fext2(2);
  F_c(2) = 0; // Fext2(3);

  F_c = f_iir_filt*F_c + (1-f_iir_filt)*F_c_prev;

  g_est = g_est + g_est_update_gain*F_c(1);

  return F_c;
}
