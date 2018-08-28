#ifndef IMP_VEL_CONTROLLER_OL_2D_RUP_H
#define IMP_VEL_CONTROLLER_OL_2D_RUP_H

#include <OL_2D_rup/Controller/Controller.h>

#include <ros/ros.h>

class ImpVelController: public Controller
{
public:
  ImpVelController(std::shared_ptr<Robot> &robot, std::shared_ptr<RefModel> &ref_model);
  ~ImpVelController() {}

  void readParams(const char *params_file = NULL);

  void init();

  void run();

  arma::vec getError();

  void update(double dt);

  bool start();

private:
  void initParams();
  void initVars();
  void estimateObjectDynamics();
  arma::vec calc_Fc_d();
  arma::vec calc_Fc();

  arma::vec Fext_filt;
  arma::mat M_rot, D_rot, K_rot;
  arma::vec S_rot, dS_rot, ddS_rot;

  bool  start_flag;


  // ========= Controller params ===========
  arma::mat M; ///< impedance controller inertia matrix
  arma::mat K; ///< impedance controller stiffness matrix
  arma::mat D; ///< impedance controller damping matrix
  double pos_tol_stop;
  double vel_tol_stop;
  double orient_ctrl_type;
  double k_click;
  double f_iir_filt; ///< iir filter gain for the force
  arma::vec F_c_prev;
  arma::vec F_c_d_prev;
  double F_neg_thres;
  arma::vec ff_gains;

  arma::vec Fext, Fext_prev;
  bool start_lifting;
  bool assume_static_obj;

  double g_est;
  double g_est_update_gain;
  double Kg;

  arma::vec Fext_dead_zone; ///< dead zone to apply to force/torqe measurement

  // ========= Controller variables ============
  arma::vec U;
  arma::vec V;
  arma::mat M_o;

  arma::vec vel_cmd;
  arma::vec Vel_cmd;

  // ========== Safety check params ============
  arma::vec vel_max;
  arma::vec F_max;
};

#endif // IMP_VEL_CONTROLLER_OL_2D_RUP_H
