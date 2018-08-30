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
  virtual void run() = 0;

  virtual void initTraining() = 0;
  virtual void logTrainData() = 0;
  virtual void train() = 0;

  double t; // current timestamp during controller execution
  arma::vec Y, dY, ddY; // produced by the target impedance model
  arma::vec f_ext; // external Cartesian force

  arma::vec g_hat; // goal estimate
  double tau_hat; // time scale estimate
  arma::mat P_theta; // covariance of g_hat and tau_hat

protected:

  std::shared_ptr<Robot> robot;
};

#endif // CONTROLLER_OL_2D_RUP_H
