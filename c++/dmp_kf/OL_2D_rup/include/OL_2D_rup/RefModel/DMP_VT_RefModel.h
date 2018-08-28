#ifndef DMP_VT_REF_MODEL_OL_2D_RUP_H
#define DMP_VT_REF_MODEL_OL_2D_RUP_H

#include <OL_2D_rup/RefModel/RefModel.h>
#include <dmp_lib/dmp_lib.h>

class DMP_VT_RefModel: public RefModel
{
  enum UpdateMethod
  {
    KF = 0, // Kalman Filter
    RLS = 1, // Recursive Least Squares
    RLWR = 2 // Recursive Locally Weighted Regression
  };

public:
  DMP_VT_RefModel() {}
  ~DMP_VT_RefModel() {}

  void init(const arma::vec &S0);
  void update(double dt, const arma::vec &model_error);
  void calcOutput(arma::vec &S_ref, arma::vec &dS_ref, arma::vec &ddS_ref);

  void initParams();
  void initVars(const arma::vec &S0);
  void updateParams(const arma::vec &model_error);
  void updateVars(double dt);

private:
  void readParams(const char *params_file = NULL);
  void expand_online(std::shared_ptr<as64_::DMP_> &dmp, arma::vec &P, arma::mat &Sigma, double sigma0);


  // ========== Model params =========
  arma::vec D_z;
  arma::vec K_z;
  double dmp_tau; // sec
  int N_kernels;
  double kernel_std_scaling;
  UpdateMethod update_method;
  arma::vec sigma_dmp_w;
  arma::vec s_n_start; // initial value of noise
  arma::vec s_n_end; // final value of noise
  double s_n_a; // rate of exponential convergence from s_n_start to end s_n_end
  double lambda;
  arma::vec k_Ferr;

  double v_thres;
  double v_a;
  double f_thres;
  double f_a;

  // ========== Model variables ============
  std::shared_ptr<as64_::CanonicalClock> canClock_ptr;
  std::shared_ptr<as64_::GatingFunction> shapeAttrGating_ptr;
  std::shared_ptr<as64_::GatingFunction> goalAttrGating_ptr;
  std::vector<std::shared_ptr<as64_::DMP_>> dmp;

  arma::vec F_err, F_err_prev;
  std::vector<arma::vec> P_w;
  std::vector<arma::mat> Sigma_w;
  arma::vec sigma_noise;
};

#endif // DMP_VT_REF_MODEL_OL_2D_RUP_H
