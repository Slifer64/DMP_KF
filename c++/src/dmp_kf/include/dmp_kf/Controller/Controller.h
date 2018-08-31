#ifndef CONTROLLER_OL_2D_RUP_H
#define CONTROLLER_OL_2D_RUP_H

#include <cstdlib>
#include <vector>
#include <memory>
#include <exception>

#include <dmp_kf/utils.h>
#include <dmp_kf/Robot/Robot.h>

class Controller
{
public:
  Controller(std::shared_ptr<Robot> &robot);
  ~Controller();

  virtual void initExecution() = 0;
  virtual void execute() = 0;

  virtual void initDemo() = 0;
  virtual void logDemoData() = 0;
  virtual void train() = 0;

  virtual bool loadTrainedModel() = 0;
  virtual bool saveTrainedModel() = 0;

  virtual void runModel() = 0;

  arma::vec q_start; ///< starting pose

  double t; // current timestamp during controller execution
  arma::vec Y, dY, ddY; // produced by the target impedance model
  arma::vec f_ext; // external Cartesian force

  arma::vec g_hat; // goal estimate
  double tau_hat; // time scale estimate
  arma::mat P_theta; // covariance of g_hat and tau_hat
  double mf; // leader-follower weight

  arma::rowvec Timed; // timestamps from demo
  arma::mat Yd_data, dYd_data, ddYd_data;

protected:

  std::shared_ptr<Robot> robot;
};

#endif // CONTROLLER_OL_2D_RUP_H
