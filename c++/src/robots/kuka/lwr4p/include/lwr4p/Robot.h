/**
 * Copyright (C) 2016 AUTH-ARL
 */

#ifndef LWR_ROBOT_LWR_ROBOT_H
#define LWR_ROBOT_LWR_ROBOT_H

#include <FastResearchInterface.h>
#include <ros/ros.h>

#include <memory>
#include <vector>
#include <string>

#include <armadillo>

namespace lwr4p
{

enum Mode {UNDEFINED = -1000, /**< For internal use */
           STOPPED = -1, /**< When the robot is stopped and does not accept commands */
           POSITION_CONTROL  = 0, /**< For sending joint position commands */
           VELOCITY_CONTROL  = 1, /**< For sending joint velocity commands */
           TORQUE_CONTROL    = 2, /**< For sending torque commands */
           IMPEDANCE_CONTROL = 3, /**< For operating in Impedance control */
           JOINT_TRAJECTORY  = 4 /**< Probably should be covered by position control */
          };

class Robot
{
public:
  explicit Robot(const char *path_to_FRI_init=NULL);
  void stop();
  void setMode(lwr4p::Mode mode);
  void waitNextCycle();
  double getControlCycle() const { return cycle; }
  Mode getMode() const { return mode; }

  arma::vec getJointPosition();

  arma::vec getJointTorque();

  arma::vec getJointExternalTorque();

  arma::mat getJacobian();

  arma::mat getTaskPose();

  arma::vec getTaskPosition();

  arma::mat getTaskOrientation();

  arma::vec getExternalWrench();


  void setJointTrajectory(const arma::vec &input, double duration);

  void setJointPosition(const arma::vec &input);

  void setJointVelocity(const arma::vec &input);

  void setJointTorque(const arma::vec &input);

  void setWrench(const arma::vec &input);

  void setTaskPose(const arma::mat &input);

  void setCartStiffness(const arma::vec &input);

  void setCartDamping(const arma::vec &input);

  bool isOk();

private:
  std::shared_ptr<FastResearchInterface> FRI;
  void startJointPositionController();
  void startJointTorqueController();
  void startCartImpController();
  void stopController();
  void saveLastJointPosition(float input[7]);
  void saveLastJointPosition();
  float last_jnt_pos[7];


  /**
   * @brief The current Mode of the robot.
   */
  Mode mode;

  /**
   * @brief The control cycle of the robot.
   *
   * Can be reading it online by the robot hardware or setted by the contructor
   * of your robot.
   */
  double cycle;

  const int N_JOINTS;

};

}  // namespace lwr4p

#endif  // LWR_ROBOT_LWR_ROBOT_H
