#include <OL_2D_rup/RefModel/DMP_VT_RefModel.h>
#include <ros/package.h>
#include <param_lib/param_lib.h>

void DMP_VT_RefModel::readParams(const char *params_file)
{
  std::string path_to_config_file;
  if (params_file != NULL) path_to_config_file = *params_file;
  else path_to_config_file = ros::package::getPath(PACKAGE_NAME)+ "/config/RefModel_config.yml";
  as64_::param_::Parser parser(path_to_config_file);

  if (!parser.getParam("D_z", D_z)) D_z = arma::vec().ones(N_DOFS)*50;
  if (!parser.getParam("K_z", K_z)) K_z = arma::vec().zeros(N_DOFS);
  if (!parser.getParam("dmp_tau", dmp_tau)) dmp_tau = 5.0;
  if (!parser.getParam("N_kernels", N_kernels)) N_kernels = 40;
  N_kernels = N_kernels * dmp_tau;
  if (!parser.getParam("kernel_std_scaling", kernel_std_scaling)) kernel_std_scaling = 1.0;

  std::string up_method;
  if (!parser.getParam("update_method", up_method)) up_method = "KF";
  if (up_method.compare("KF")) update_method = UpdateMethod::KF;
  else if (up_method.compare("RLS")) update_method = UpdateMethod::RLS;
  else if (up_method.compare("RLWR")) update_method = UpdateMethod::RLWR;
  else throw std::invalid_argument("DMP_VT_RefModel::readParams: Unknown update method: \"" + up_method + "\"\n");

  if (!parser.getParam("sigma_dmp_w", sigma_dmp_w)) sigma_dmp_w = arma::vec().ones(N_DOFS)*150;
  if (!parser.getParam("s_n_start", s_n_start)) s_n_start = arma::vec().ones(N_DOFS)*0.001;
  if (!parser.getParam("s_n_end", s_n_end)) s_n_end = arma::vec().ones(N_DOFS)*0.0001;
  if (!parser.getParam("s_n_a", s_n_a)) s_n_a = 0.01;
  if (!parser.getParam("lambda", lambda)) lambda = 0.995;
  if (!parser.getParam("k_Ferr", k_Ferr)) k_Ferr = arma::vec().ones(N_DOFS)*0.1;

  if (!parser.getParam("v_thres", v_thres)) v_thres = 0.0;
  if (!parser.getParam("v_a", v_a)) v_a = 10.0;
  if (!parser.getParam("f_thres", f_thres)) f_thres = 0.0;
  if (!parser.getParam("f_a", f_a)) f_a = 10.0;
}

void DMP_VT_RefModel::init(const arma::vec &S0)
{
  initParams();
  initVars(S0);
}

void DMP_VT_RefModel::update(double dt, const arma::vec &model_error)
{
  updateParams(model_error);
  updateVars(dt);
}

void DMP_VT_RefModel::calcOutput(arma::vec &S_ref, arma::vec &dS_ref, arma::vec &ddS_ref)
{
  ddS.zeros(N_DOFS);
  arma::vec states_dot;
  for (int i=0;i<N_DOFS;i++)
  {
    states_dot = dmp[i]->getStatesDot(x, S(i), dS(i), 0, 0, dS_c(i), ddS_c(i));
    ddS(i) = states_dot(0);
  }
  dx = canClock_ptr->getPhaseDot(x);

  S_ref = S;
  dS_ref = dS;
  ddS_ref = ddS;
}

void DMP_VT_RefModel::initParams()
{
  readParams();

  canClock_ptr = as64_::getCanClock("lin", dmp_tau);
  shapeAttrGating_ptr = as64_::getGatingFun("constant", 1.0, 1.0);
  goalAttrGating_ptr = as64_::getGatingFun("constant", 1.0, 1.0);

  dmp.resize(N_DOFS);
  for (int i=0;i<dmp.size();i++)
  {
    dmp[i].reset(new as64_::DMP_VT(N_kernels, D_z(i), K_z(i)/D_z(i), canClock_ptr, shapeAttrGating_ptr, goalAttrGating_ptr, kernel_std_scaling));
  }
}

void DMP_VT_RefModel::initVars(const arma::vec &S0)
{
  S = S0;
  dS.zeros(N_DOFS);
  ddS.zeros(N_DOFS);
  x = 0.0;
  dx = 0.0;

  dS_c.zeros(N_DOFS);
  ddS_c.zeros(N_DOFS);

  F_err_prev.zeros(N_DOFS);
  F_err.zeros(N_DOFS);

  P_w.resize(N_DOFS);
  Sigma_w.resize(N_DOFS);
  for (int i=0;i<P_w.size();i++)
  {
    P_w[i] = sigma_dmp_w(i)*arma::vec().ones(N_kernels);
    Sigma_w[i] = arma::diagmat(P_w[i]);
  }

  sigma_noise = s_n_start;
}

void DMP_VT_RefModel::updateParams(const arma::vec &model_error)
{
  double v = std::fabs(dS(1)); // z velocity
  double f = std::fabs(model_error(1)); // z error
  double v_gain = 1 / ( 1 + std::exp(v_a*(v_thres - v)) );
  double f_gain = 1 / ( 1 + std::exp(f_a*(f_thres - f)) );

  // static int counter;
  // counter++;
  //
  // if (counter == 100)
  // {
  //   std::cout << "v = " << v << "\n";
  //   std::cout << "v_gain = " << v_gain << "\n";
  //   std::cout << "f = " << f << "\n";
  //   std::cout << "f_gain = " << f_gain << "\n";
  //   counter = 0;
  // }

  arma::vec k_f = k_Ferr;
  k_f(1) = k_Ferr(1) * (v_gain + f_gain)/2;

  // Force error
  F_err_prev = F_err;
  F_err = k_f % model_error;
  // sigma_noise = 0.1*abs(F_err-F_err_prev).^2;

  // Model adaption
  for (int i=0;i<N_DOFS;i++)
  {
    switch (update_method)
    {
      case KF:
        dmp[i]->update_weights_with_KF(x, F_err(i), 0.0, 0.0, Sigma_w[i], sigma_noise(i));
        sigma_noise = (1-s_n_a)*sigma_noise + s_n_a*s_n_end;
        break;
      case RLS:
        dmp[i]->update_weights_with_RLS(x, F_err(i), 0.0, 0.0, Sigma_w[i], lambda);
        break;
      case RLWR:
        dmp[i]->update_weights_with_RLWR(x, F_err(i), 0.0, 0.0, Sigma_w[i], lambda);
        break;
      default:
        throw std::runtime_error("DMP_VT_RefModel::updateModelParams: Unsupported update method...\n");
    }
    P_w[i] = arma::diagvec(Sigma_w[i]);

    this->expand_online(dmp[i], P_w[i], Sigma_w[i], sigma_dmp_w(i));
  }

}

void DMP_VT_RefModel::expand_online(std::shared_ptr<as64_::DMP_> &dmp, arma::vec &P, arma::mat &Sigma, double sigma0)
{
  int N = dmp->w.size();
  double c_new = 2*dmp->c(N-1)-dmp->c(N-2);
  arma::vec psi = dmp->kernelFunction(this->x);
  if (psi(0) < 1e-50)
  {
    dmp->w.subvec(0,N-2) = dmp->w.subvec(1,N-1);
    dmp->w(N-1) = 0;

    dmp->c.subvec(0,N-2) = dmp->c.subvec(1,N-1);
    dmp->c(N-1) = c_new;

    P.subvec(0,N-2) = P.subvec(1,N-1);
    P(N-1) = sigma0;

    Sigma.submat(0,0,N-2,N-2) = Sigma.submat(1,1,N-1,N-1);
    Sigma.row(N-1) = arma::rowvec().zeros(N);
    Sigma.col(N-1) = arma::vec().zeros(N);
    Sigma(N-1,N-1) = sigma0;
  }
}

void DMP_VT_RefModel::updateVars(double dt)
{
  x = x + dx*dt;
  S = S + dS*dt;
  dS = dS + ddS*dt;
}
