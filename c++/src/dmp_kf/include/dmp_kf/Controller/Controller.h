#ifndef CONTROLLER_OL_2D_RUP_H
#define CONTROLLER_OL_2D_RUP_H

#include <cstdlib>
#include <vector>
#include <memory>
#include <exception>

#include <dmp_kf/utils.h>
#include <dmp_kf/Robot/Robot.h>
#include <dmp_kf/GUI/GUI.h>

#include <dmp_kf/LogData.h>

class Controller
{
public:

  Controller(std::shared_ptr<Robot> &robot, std::shared_ptr<GUI> &gui);
  ~Controller();

  virtual void initExecution() = 0;
  virtual void execute() = 0;
  bool saveExecutionData(std::string &err_msg);

  virtual bool simulate() = 0;
  bool saveSimulationData(std::string &err_msg);

  virtual void initDemo() = 0;
  virtual void logDemoData() = 0;
  virtual bool train(std::string &err_msg) = 0;

  virtual bool loadTrainedModel(std::string &err_msg) = 0;
  virtual bool saveTrainedModel(std::string &err_msg) = 0;
  bool saveTrainingData(std::string &err_msg);
  bool loadTrainingData(std::string &err_msg);

  virtual bool runModel() = 0;
  bool saveModelRunData(std::string &err_msg);

  void setStartPose();

  std::string getErrMsg() const { return err_msg; }
  void setErrMsg(const std::string &msg) { err_msg = msg; }

  double mixingFun(double f);
  void initMixingFun(double x1, double x2);
  double p_5th[6];
  double f1_, f2_;

  bool is_trained;

  arma::vec q_start; ///< starting pose
  arma::vec y_g; ///< goal position

  double t; // current timestamp during controller execution
  arma::vec Y, dY, ddY; // produced by the target impedance model
  arma::vec f_ext; // external Cartesian force
  arma::vec f_ext_raw;

  arma::vec g_hat; // goal estimate
  double tau_hat; // time scale estimate
  arma::mat P_theta; // covariance of g_hat and tau_hat
  double mf; // leader-follower weight

  KinematicData modelRun_data;

  KinematicData train_data;
  arma::vec g_d; // goal position from demo used for initializing g_hat
  double tau_d; // time scale from demo used for initializing tau_hat

  ExecutionData exec_data;

  arma::vec g_scale;
  double tau_scale;
  ExecutionData sim_data;
protected:

  std::string err_msg;

  std::shared_ptr<Robot> robot;
  std::shared_ptr<GUI> gui;
};

#endif // CONTROLLER_OL_2D_RUP_H
