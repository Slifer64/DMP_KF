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
  ~Robot();
  void stop() { FRI->StopRobot(); }
  bool isOk() { return FRI->IsMachineOK(); }
  void setMode(lwr4p::Mode mode);
  void waitNextCycle() { FRI->WaitForKRCTick(); }
  double getControlCycle() const { return cycle; }
  Mode getMode() const { return mode; }

  arma::vec getJointPosition()
  {
    arma::vec output(N_JOINTS);
    static float temp[7];
    FRI->GetMeasuredJointPositions(temp);
    for (size_t i = 0; i < N_JOINTS; i++) {
      output(i) = temp[i];
    }
    return output;
  }

  arma::vec getJointTorque()
  {
    arma::vec output(N_JOINTS);
    static float joint_torques[7];
    FRI->GetMeasuredJointTorques(joint_torques);
    output(0) = joint_torques[0];
    output(1) = joint_torques[1];
    output(2) = joint_torques[2];
    output(3) = joint_torques[3];
    output(4) = joint_torques[4];
    output(5) = joint_torques[5];
    output(6) = joint_torques[6];

    return output;
  }

  arma::vec getJointExternalTorque()
  {
    arma::vec output(N_JOINTS);
    static float estimated_external_joint_torques[7];
    FRI->GetEstimatedExternalJointTorques(estimated_external_joint_torques);
    output(0) = estimated_external_joint_torques[0];
    output(1) = estimated_external_joint_torques[1];
    output(2) = estimated_external_joint_torques[2];
    output(3) = estimated_external_joint_torques[3];
    output(4) = estimated_external_joint_torques[4];
    output(5) = estimated_external_joint_torques[5];
    output(6) = estimated_external_joint_torques[6];

    return output;
  }

  arma::mat getEEJacobian()
  {
    arma::mat output(6, N_JOINTS);

    FRI->GetCurrentJacobianMatrix(jacob_temp);
    std::vector<int> jac_indexes{0, 1, 2, 5, 4, 3};
    for (size_t i = 0; i < jac_indexes.size(); i++)
    {
      for (size_t j = 0; j < 7; j++)
      {
        output(i, j) = jacob_temp[jac_indexes[i]][j];
      }
    }

    return output;
  }

  arma::mat getRobotJacobian()
  {
    arma::mat output = getEEJacobian();
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    arma::mat rot;
    rot << temp[0] << temp[1] << temp[2] << arma::endr
        << temp[4] << temp[5] << temp[6] << arma::endr
        << temp[8] << temp[9] << temp[10];
    output.submat(0, 0, 2, 6) = rot * output.submat(0, 0, 2, 6);
    output.submat(3, 0, 5, 6) = rot * output.submat(3, 0, 5, 6);

    return output;
  }

  arma::mat getTaskPose()
  {
    arma::mat output = arma::mat().eye(4, 4);
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
        output(i, j) = temp[i * 4 + j];
      }
    }

    return output;
  }

  arma::vec getTaskPosition()
  {
    arma::vec output(3);
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    output(0) = temp[3];
    output(1) = temp[N_JOINTS];
    output(2) = temp[11];

    return output;
  }

  arma::mat getTaskOrientation()
  {
    arma::mat output(3,3);
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    output(0, 0) = temp[0];
    output(0, 1) = temp[1];
    output(0, 2) = temp[2];
    output(1, 0) = temp[4];
    output(1, 1) = temp[5];
    output(1, 2) = temp[6];
    output(2, 0) = temp[8];
    output(2, 1) = temp[9];
    output(2, 2) = temp[10];

    return output;
  }

  arma::vec getExternalWrench()
  {
    arma::vec output(6);
    static float estimated_external_cart_forces_and_torques[6];
    FRI->GetEstimatedExternalCartForcesAndTorques(estimated_external_cart_forces_and_torques);
    output(0) = estimated_external_cart_forces_and_torques[0];
    output(1) = estimated_external_cart_forces_and_torques[1];
    output(2) = estimated_external_cart_forces_and_torques[2];
    output(3) = estimated_external_cart_forces_and_torques[3];
    output(4) = estimated_external_cart_forces_and_torques[4];
    output(5) = estimated_external_cart_forces_and_torques[5];

    return output;
  }

  void setJointPosition(const arma::vec &input)
  {
    if (this->mode == lwr4p::Mode::POSITION_CONTROL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[JointPosController::setJointPosition] Joint positions are commanded with closed controller.\n");
        printf("Opening controller ...\n");
        // start te cotroller
        startJointPositionController();
        // wait one tick
        FRI->WaitForKRCTick();
      }
      // temp variables
      static float temp[7];
      // put the values from arma to float[]
      for (int i = 0; i < 7; i++) {
        temp[i] = input(i);
      }
      // set commanded joint positions to qd
      FRI->SetCommandedJointPositions(temp);
      saveLastJointPosition(temp);
    }
    else
    {
      std::cerr << "setJointPosition only available in POSITION_CONTROL mode" << std::endl;
    }
  }

  void setJointVelocity(const arma::vec &input)
  {
    if (this->mode == lwr4p::Mode::VELOCITY_CONTROL)
    {
      static float temp[7];
      for (size_t i = 0; i < 7; i++)
      {
        last_jnt_pos[i] += input(i) * cycle;
      }
      FRI->SetCommandedJointPositions(last_jnt_pos);
    }
    else
    {
      std::cerr << "setJointVelocity only available in VELOCITY_CONTROL mode" << std::endl;
    }
  }

  void setJointTorque(const arma::vec &input)
  {
    if (this->mode == lwr4p::Mode::TORQUE_CONTROL)
    {
      static float torques[7];
      for (size_t i = 0; i < 7; i++)
      {
        torques[i] = input(i);
      }
      FRI->SetCommandedJointTorques(torques);
      // Mirror the joint positions and the cartesian pose in order to avoid
      // cartesian deviation errors
      float temp_position[N_JOINTS];
      FRI->GetMeasuredJointPositions(temp_position);
      FRI->SetCommandedJointPositions(temp_position);
      static float temp_pose[12];
      FRI->GetMeasuredCartPose(temp_pose);
      FRI->SetCommandedCartPose(temp_pose);

      saveLastJointPosition(temp_position);
    }
    else
    {
      std::cerr << "setJointTorque only available in TORQUE_CONTROL mode" << std::endl;
    }
  }

  void setTaskPose(const arma::mat &input)
  {
    if (this->mode == lwr4p::Mode::IMPEDANCE_CONTROL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[CartImpedanceController::setTaskPose] Cartesian wrench is commanded with closed controller.\n");
        printf("Opening controller ...\n");
        // start te controller
        startCartImpController();
        // wait one tick
        FRI->WaitForKRCTick();
      }
      // temp variables
      static float temp[12];
      for (size_t i = 0; i < 3; i++)
        {
          for (size_t j = 0; j < 4; j++)
          {
            temp[i * 4 + j] = input(i, j);
          }
      }

      // set commanded task pose
      FRI->SetCommandedCartPose(temp);
    }
    else
    {
      std::cerr << "setTaskPose only available in IMPEDANCE_CONTROL mode" << std::endl;
    }
  }

  void setWrench(const arma::vec &input)
  {
    if (this->mode == lwr4p::Mode::IMPEDANCE_CONTROL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[CartImpedanceController::setWrench] Cartesian wrench is commanded with closed controller.\n");
        printf("Opening controller ...\n");
        // start te controller
        startCartImpController();
        // wait one tick
        FRI->WaitForKRCTick();
      }
      // temp variables
      static float temp[6];
      // put the values from arma to float[]
      for (int i = 0; i < 6; i++) {
        temp[i] = input(i);
      }

      // set commanded Cartesian forces/torques
      FRI->SetCommandedCartForcesAndTorques(temp);

      float temp_position[N_JOINTS];
      FRI->GetMeasuredJointPositions(temp_position);
      // FRI->SetCommandedJointPositions(temp_position);
      // float temp_pose[12];
      // FRI->GetMeasuredCartPose(temp_pose);
      // FRI->SetCommandedCartPose(temp_pose);
      saveLastJointPosition(temp_position); //needed for numeric differentation to obtain q_dot [isn't it?]

    }
    else
    {
      std::cerr << "setWrench only available in IMPEDANCE_CONTROL mode" << std::endl;
    }
  }

  void setJointTrajectory(const arma::vec &input, double duration);

  void setCartStiffness(const arma::vec &input);

  void setCartDamping(const arma::vec &input);

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


  float **jacob_temp;

};


}  // namespace lwr4p

#endif  // LWR_ROBOT_LWR_ROBOT_H
