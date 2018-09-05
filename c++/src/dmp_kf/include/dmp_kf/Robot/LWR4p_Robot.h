#ifndef LWR4p_Robot_OL_2D_RUP_H
#define LWR4p_Robot_OL_2D_RUP_H

#include <dmp_kf/Robot/Robot.h>

#include <lwr4p/Robot.h>

#include <ati_sensor/ft_sensor.h>

class LWR4p_Robot: public Robot
{
public:
  LWR4p_Robot();
  ~LWR4p_Robot();

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

  arma::vec getBaseWrench() const;

private:
  const int N_JOINTS;
  std::shared_ptr<lwr4p::Robot> robot;
  ati::FTSensor ftsensor;

};

#endif // LWR4p_Robot_OL_2D_RUP_H
