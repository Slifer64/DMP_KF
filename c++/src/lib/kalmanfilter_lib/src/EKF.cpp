#include <kalmanfilter_lib/EKF.h>

namespace as64_
{

namespace kf_
{

EKF::EKF(const arma::vec &theta0, const arma::mat &P0, int N_msr,
    arma::vec (*stateTransFun_ptr)(const arma::vec &theta, void *cookie),
    arma::vec (*msrFun_ptr)(const arma::vec &theta, void *cookie))
{
  this->init(theta0, P0, N_msr);

  this->stateTransFun_ptr = std::bind(stateTransFun_ptr, std::placeholders::_1, std::placeholders::_2);
  this->msrFun_ptr = std::bind(msrFun_ptr, std::placeholders::_1, std::placeholders::_2);
}

void EKF::init(const arma::vec &theta0, const arma::mat &P0, int N_msr)
{
  this->theta = theta0;
  this->P = P0;

  int N_params = theta0.size();

  this->setProcessNoiseCov(arma::mat().eye(N_params,N_params)*1e-5);
  this->setMeasureNoiseCov(arma::mat().eye(N_msr,N_msr)*0.05);

  this->setFadingMemoryCoeff(1.0);

  this->enableParamsContraints(false);
  this->setParamsConstraints(arma::mat().zeros(1,N_params),arma::vec().zeros(1));

  this->setPartDerivStep(0.001);

  this->stateTransFunJacob_ptr = std::bind(&EKF::calcStateTransFunJacob, *this, std::placeholders::_1, std::placeholders::_2);
  this->msrFunJacob_ptr = std::bind(&EKF::calcMsrFunJacob, *this, std::placeholders::_1, std::placeholders::_2);
}

void EKF::setFadingMemoryCoeff(double a_p)
{
  this->a_p = a_p;
}

void EKF::enableParamsContraints(bool enable_contraints)
{
  this->enable_constraints = enable_contraints;
}

void EKF::setParamsConstraints(const arma::mat &A_c, const arma::vec &b_c)
{
  this->A_c = A_c;
  this->b_c = b_c;
}

void EKF::setProcessNoiseCov(const arma::mat &Q)
{
  this->Q = Q;
}

void EKF::setMeasureNoiseCov(const arma::mat &R)
{
  this->R = R;
}

void EKF::setPartDerivStep(double dtheta)
{
  this->dtheta = arma::vec().ones(this->theta.size())*dtheta;
}

void EKF::setPartDerivStep(const arma::vec &dtheta)
{
  this->dtheta = dtheta;
}

void EKF::predict(void *cookie)
{
  F_k = stateTransFunJacob_ptr(theta, cookie);
  theta = stateTransFun_ptr(theta, cookie);
  P = std::pow(a_p,2)*F_k*P*F_k.t() + Q;
}

void EKF::correct(const arma::vec &z, void *cookie)
{
  // =====  Retrive the measurement function Jacobian  =====
  H_k = msrFunJacob_ptr(theta, cookie);

  // =====  Correction estimates =====
  arma::vec z_hat = msrFun_ptr(theta, cookie);
  arma::mat K_kf = arma::solve((H_k*P*H_k.t() + R).t(), H_k*P.t(), arma::solve_opts::fast).t();
  // arma::mat K_kf = P*H_k.t()*arma::inv(H_k*P*H_k.t() + R);
  // arma::mat K_kf = P*H_k.t()*arma::inv_sympd(H_k*P*H_k.t() + R);


  // arma::mat K_kf2 = P*H_k.t()*arma::inv(H_k*P*H_k.t() + R);
  // arma::mat K_kf3 = P*H_k.t()*arma::inv_sympd(H_k*P*H_k.t() + R);
  //
  // double err2 = arma::norm(arma::diagvec(K_kf) - arma::diagvec(K_kf2));
  // double err3 = arma::norm(arma::diagvec(K_kf) - arma::diagvec(K_kf3));
  //
  // if (err2 > 1e-6)
  // {
  //   std::cout << "============================\n";
  //   std::cout << "err2 = " << err2 << "\n";
  //   std::cout << "K_kf = \n" << K_kf << "\n";
  //   std::cout << "K_kf2 = \n" << K_kf2 << "\n";
  //   std::cout << "============================\n";
  // }
  //
  // if (err3 > 1e-6)
  // {
  //   std::cout << "============================\n";
  //   std::cout << "err3 = " << err3 << "\n";
  //   std::cout << "K_kf = \n" << K_kf << "\n";
  //   std::cout << "K_kf3 = \n" << K_kf3 << "\n";
  //   std::cout << "============================\n";
  // }

  theta = theta + K_kf * (z - z_hat);

  // =====  Apply projection if enabled  =====
  arma::mat D = arma::mat(A_c.n_rows, A_c.n_cols); // active contraints
  arma::vec d(b_c.size());
  bool proj_flag = false;
  if (enable_constraints)
  {
    int j = -1;
    for (int i=0; i<A_c.n_rows; i++)
    {
      if (arma::dot(A_c.row(i),theta) > b_c(i))
      {
        j++;
        d(j) = b_c(i);
        D.row(j) = A_c.row(i);
      }
    }
    if (j >= 0)
    {
      proj_flag = true;
      D = D.rows(0,j);
      d = d.rows(0,j);
    }
  }

  int N_params = theta.size();
  arma::mat I = arma::mat().eye(N_params, N_params);

  if (proj_flag)
  {
    K_kf = ( I - P*D.t()*arma::inv_sympd(D*P*D.t())*D ) * K_kf;
    theta = theta - P*D.t()*arma::inv_sympd(D*P*D.t())*(D*theta-d);
    //K_kf = ( I - D.t()*arma::inv_sympd(D*D.t())*D ) * K_kf;
    //theta = theta - D.t()*arma::inv_sympd(D*D.t())*(D*theta-d);
  }

  // =====  Calculate new covariance  =====
  P = (I - K_kf*H_k) * P * (I - K_kf*H_k).t() + K_kf*R*K_kf.t();

  K = K_kf;
}

arma::mat EKF::calcStateTransFunJacob(const arma::vec &theta, void *cookie)
{
  int N_params = theta.size();
  // compute Jacobian numerically
  arma::mat F_k = arma::mat().zeros(N_params,N_params);
  arma::vec dtheta_j = arma::vec().zeros(N_params);
  for (int j=0; j<N_params; j++)
  {
      dtheta_j(j) = this->dtheta(j);
      arma::vec Ftheta2 = this->stateTransFun_ptr(theta + dtheta_j, cookie);
      arma::vec Ftheta1 = this->stateTransFun_ptr(theta - dtheta_j, cookie);
      F_k.col(j) = (Ftheta2 - Ftheta1) / (2*this->dtheta(j));
      dtheta_j(j) = 0.0;
  }

  return F_k;
}

arma::mat EKF::calcMsrFunJacob(const arma::vec &theta, void *cookie)
{
  int N_out = this->R.n_rows;
  int N_params = theta.size();
  // compute Jacobian numerically
  arma::mat H_k = arma::mat().zeros(N_out,N_params);
  arma::vec dtheta_j = arma::vec().zeros(N_params);
  for (int j=0; j<N_params; j++)
  {
      dtheta_j(j) = this->dtheta(j);
      arma::vec Htheta2 = this->msrFun_ptr(theta + dtheta_j, cookie);
      arma::vec Htheta1 = this->msrFun_ptr(theta - dtheta_j, cookie);
      H_k.col(j) = (Htheta2 - Htheta1) / (2*this->dtheta(j));
      dtheta_j(j) = 0.0;
  }

  return H_k;
}

} // namespace kf_

} // namespace as64_
