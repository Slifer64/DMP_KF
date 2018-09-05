#ifndef UR10_ROBOT_OL_2D_RUP_H
#define UR10_ROBOT_OL_2D_RUP_H

#include <dmp_kf/Robot/Robot.h>
#include <ur10/Robot.h>

class UR10_Robot: public Robot
{
public:
  UR10_Robot();
  ~UR10_Robot();

  void init();

  double getControlCycle() const;

  void update();

  void command();

  void stop();

  bool isOk();

  void setMode(const Robot::Mode &mode);

  void setTaskVelocity(const arma::vec &vel);

  void setJointTrajectory(const arma::vec &qT, double duration);
  arma::vec getJointPosition() const;

  arma::mat getTaskPose() const;
  arma::vec getTaskPosition() const;
  arma::vec getTaskOrientation() const;

  arma::vec getTaskWrench();

private:
  std::shared_ptr<ur10_::Robot> robot;

};

#endif // UR10_ROBOT_OL_2D_RUP_H
