#ifndef CONTROLLER_OL_2D_RUP_H
#define CONTROLLER_OL_2D_RUP_H

#include <cstdlib>
#include <vector>
#include <memory>
#include <exception>

#include <ros/package.h>
#include <armadillo>

#include <OL_2D_rup/utils.h>
#include <OL_2D_rup/Robot/Robot.h>
#include <OL_2D_rup/RefModel/RefModel.h>

class Controller
{
public:
  Controller(std::shared_ptr<Robot> &robot, std::shared_ptr<RefModel> &model);
  ~Controller();

  virtual void init() = 0;

  virtual void run() = 0;

  // ========= Controller variables ============
  arma::vec S;
  arma::vec dS;
  arma::vec ddS;
  arma::vec F_c;
  arma::vec F_c_d;

  arma::vec S_ref, dS_ref, ddS_ref;

  bool is_okay;

protected:

  std::shared_ptr<Robot> robot;
  std::shared_ptr<RefModel> ref_model;

  double m_o;
  double max_m_o; ///< maximum weight carried by the robot
  double max_torque; ///< maximum torque that the robot is allowed to feeel during initialization
  arma::vec r_CoM; ///< initial estimation of object's CoM from robot end-effector
};

#endif // CONTROLLER_OL_2D_RUP_H
