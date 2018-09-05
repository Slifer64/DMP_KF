#ifndef Sim_Robot_OL_2D_RUP_H
#define Sim_Robot_OL_2D_RUP_H

#include <dmp_kf/Robot/Robot.h>
#include <robot_sim/robot_sim.h>

class Sim_Robot: public Robot
{
public:
  Sim_Robot();
  ~Sim_Robot();

  void init();

  double getControlCycle() const;

  void update();

  void command();

  void stop();

  bool isOk();

  void enable();

  void setMode(const Robot::Mode &mode);

  void setTaskVelocity(const arma::vec &vel);

  void setJointTrajectory(const arma::vec &qT, double duration);
  arma::vec getJointPosition() const;

  arma::mat getTaskPose() const;
  arma::vec getTaskPosition() const;
  arma::vec getTaskOrientation() const;

  arma::vec getTaskWrench();

  arma::vec getBaseWrench() const;

private:
  int N_JOINTS;
  std::shared_ptr<as64_::RobotSim> robot;

};

#endif // Sim_Robot_OL_2D_RUP_H
