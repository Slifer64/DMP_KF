/**
 * Copyright (C) 2016 AUTH-ARL
 */

#include <lwr_robot/lwr_robot.h>
#include <iostream>
#include <string>

#define JOINT_SIZE 7

namespace lwr
{
namespace robot
{
Robot::Robot(std::shared_ptr<arl::robot::Model> m, const std::string& name) :
  arl::robot::Robot(m, name)
{
  FRI.reset(new FastResearchInterface("/home/user/lwr/980500-FRI-Driver.init"));
  cycle = static_cast<double>(FRI->GetFRICycleTime());
  state[0].cmd = state[0].msr;
  this->mode = arl::robot::Mode::UNDEFINED;  // Dummy initialization before stopping controller
  stopController();  // Initially the robot is stopped
  startJointPositionController();
}

void Robot::waitNextCycle()
{
  FRI->WaitForKRCTick();
}

void Robot::setMode(arl::robot::Mode mode, int chain_index)
{
  switch (mode)
  {
    case arl::robot::Mode::STOPPED :
      if (this->mode != arl::robot::Mode::STOPPED)
      {
        stopController();
      }
      saveLastJointPosition();
      break;
    case arl::robot::Mode::POSITION_CONTROL :
      if (this->mode != arl::robot::Mode::POSITION_CONTROL)
      {
        if (this->mode != arl::robot::Mode::STOPPED)
        {
          stopController();
        }
        startJointPositionController();
        saveLastJointPosition();
      }
      break;
    case arl::robot::Mode::VELOCITY_CONTROL :
      if (this->mode != arl::robot::Mode::VELOCITY_CONTROL)
      {
        if (this->mode != arl::robot::Mode::POSITION_CONTROL)
        {
          stopController();
          startJointPositionController();
        }
        this->mode = arl::robot::Mode::VELOCITY_CONTROL;
        saveLastJointPosition();
      }
      break;
    case arl::robot::Mode::TORQUE_CONTROL :
      if (this->mode != arl::robot::Mode::TORQUE_CONTROL)
      {
        if (this->mode != arl::robot::Mode::STOPPED)
        {
          stopController();
        }
        startJointTorqueController();
        saveLastJointPosition();
      }
      break;
    case arl::robot::Mode::IMPEDANCE_CONTROL :
      if (this->mode != arl::robot::Mode::IMPEDANCE_CONTROL)
      {
        if (this->mode != arl::robot::Mode::STOPPED)
        {
          stopController();
        }
        startCartImpController();
        saveLastJointPosition();
      }
      break;
    default: std::cout << "Mode " << mode << " Not available" << std::endl;
  }
}

void Robot::setJointTrajectory(const KDL::JntArray &input, double duration, const int chain_index)
{
  setJointTrajectoryTemplate(input, duration, chain_index);
}
void Robot::setJointTrajectory(const arma::vec &input, double duration, const int chain_index)
{
  setJointTrajectoryTemplate(input, duration, chain_index);
}
void Robot::setJointTrajectory(const Eigen::VectorXd &input, double duration, const int chain_index)
{
  setJointTrajectoryTemplate(input, duration, chain_index);
}

void Robot::setJointPosition(const KDL::JntArray &input, const int chain_index)
{
  setJointPositionTemplate(input, chain_index);
}
void Robot::setJointPosition(const arma::vec &input, const int chain_index)
{
  setJointPositionTemplate(input, chain_index);
}
void Robot::setJointPosition(const Eigen::VectorXd &input, const int chain_index)
{
  setJointPositionTemplate(input, chain_index);
}

void Robot::setJointVelocity(const KDL::JntArray &input, const int chain_index)
{
  setJointVelocityTemplate(input, chain_index);
}
void Robot::setJointVelocity(const arma::vec &input, const int chain_index)
{
  setJointVelocityTemplate(input, chain_index);
}
void Robot::setJointVelocity(const Eigen::VectorXd &input, const int chain_index)
{
  setJointVelocityTemplate(input, chain_index);
}

void Robot::setJointTorque(const KDL::JntArray &input, const int chain_index)
{
  setJointTorqueTemplate(input, chain_index);
}
void Robot::setJointTorque(const arma::vec &input, const int chain_index)
{
  setJointTorqueTemplate(input, chain_index);
}
void Robot::setJointTorque(const Eigen::VectorXd &input, const int chain_index)
{
  setJointTorqueTemplate(input, chain_index);
}

void Robot::setTaskPose(const KDL::Frame &input, const int chain_index)
{
  setTaskPoseTemplate(input, chain_index);
}
void Robot::setTaskPose(const arma::mat &input, const int chain_index)
{
  setTaskPoseTemplate(input, chain_index);
}
void Robot::setTaskPose(const Eigen::MatrixXd &input, const int chain_index)
{
  setTaskPoseTemplate(input, chain_index);
}

void Robot::setCartStiffness(const KDL::Wrench &input, const int chain_index)
{
  setCartStiffnessTemplate(input, chain_index);
}
void Robot::setCartStiffness(const arma::vec &input, const int chain_index)
{
  setCartStiffnessTemplate(input, chain_index);
}
void Robot::setCartStiffness(const Eigen::VectorXd &input, const int chain_index)
{
  setCartStiffnessTemplate(input, chain_index);
}

void Robot::setCartDamping(const KDL::Wrench &input, const int chain_index)
{
  setCartDampingTemplate(input, chain_index);
}
void Robot::setCartDamping(const arma::vec &input, const int chain_index)
{
  setCartDampingTemplate(input, chain_index);
}
void Robot::setCartDamping(const Eigen::VectorXd &input, const int chain_index)
{
  setCartDampingTemplate(input, chain_index);
}

void Robot::getJointPosition(KDL::JntArray &output, const int chain_index)
{
  getJointPositionTemplate(output, chain_index);
}
void Robot::getJointPosition(arma::vec &output, const int chain_index)
{
  getJointPositionTemplate(output, chain_index);
}
void Robot::getJointPosition(Eigen::VectorXd &output, const int chain_index)
{
  getJointPositionTemplate(output, chain_index);
}

void Robot::getJointVelocity(KDL::JntArray &output, const int chain_index)
{
  getJointVelocityTemplate(output, chain_index);
}
void Robot::getJointVelocity(arma::vec &output, const int chain_index)
{
  getJointVelocityTemplate(output, chain_index);
}
void Robot::getJointVelocity(Eigen::VectorXd &output, const int chain_index)
{
  getJointVelocityTemplate(output, chain_index);
}

void Robot::getJointTorque(KDL::JntArray &output, const int chain_index)
{
  getJointTorqueTemplate(output, chain_index);
}
void Robot::getJointTorque(arma::vec &output, const int chain_index)
{
  getJointTorqueTemplate(output, chain_index);
}
void Robot::getJointTorque(Eigen::VectorXd &output, const int chain_index)
{
  getJointTorqueTemplate(output, chain_index);
}

void Robot::getJointExternalTorque(KDL::JntArray &output, const int chain_index)
{
  getJointExternalTorqueTemplate(output, chain_index);
}
void Robot::getJointExternalTorque(arma::vec &output, const int chain_index)
{
  getJointExternalTorqueTemplate(output, chain_index);
}
void Robot::getJointExternalTorque(Eigen::VectorXd &output, const int chain_index)
{
  getJointExternalTorqueTemplate(output, chain_index);
}

void Robot::getJacobian(KDL::Jacobian &output, const int chain_index)
{
  output.resize(this->model->getNrOfJoints(chain_index));
  getJacobianTemplate(output, chain_index);
  static float temp[12];
  FRI->GetMeasuredCartPose(temp);
  KDL::Rotation kdl_rot = KDL::Rotation(temp[0], temp[1], temp[2],
                                        temp[4], temp[5], temp[6],
                                        temp[8], temp[9], temp[10]);
  Eigen::Matrix3d rot;
  rot << kdl_rot.data[0], kdl_rot.data[1], kdl_rot.data[2],
         kdl_rot.data[3], kdl_rot.data[4], kdl_rot.data[5],
         kdl_rot.data[6], kdl_rot.data[7], kdl_rot.data[8];
  output.data.block(0, 0, 3, 7) = rot * output.data.block(0, 0, 3, 7);
  output.data.block(3, 0, 3, 7) = rot * output.data.block(3, 0, 3, 7);
}
void Robot::getJacobian(arma::mat &output, const int chain_index)
{
  output.resize(6, this->model->getNrOfJoints(chain_index));
  getJacobianTemplate(output, chain_index);
  static float temp[12];
  FRI->GetMeasuredCartPose(temp);
  arma::mat rot;
  rot << temp[0] << temp[1] << temp[2] << arma::endr
      << temp[4] << temp[5] << temp[6] << arma::endr
      << temp[8] << temp[9] << temp[10];
  output.submat(0, 0, 2, 6) = rot * output.submat(0, 0, 2, 6);
  output.submat(3, 0, 5, 6) = rot * output.submat(3, 0, 5, 6);
}
void Robot::getJacobian(Eigen::MatrixXd &output, const int chain_index)
{
  output.resize(6, this->model->getNrOfJoints(chain_index));
  getJacobianTemplate(output, chain_index);
  static float temp[12];
  FRI->GetMeasuredCartPose(temp);
  KDL::Rotation kdl_rot = KDL::Rotation(temp[0], temp[1], temp[2],
                                        temp[4], temp[5], temp[6],
                                        temp[8], temp[9], temp[10]);
  Eigen::Matrix3d rot;
  rot << kdl_rot.data[0], kdl_rot.data[1], kdl_rot.data[2],
         kdl_rot.data[3], kdl_rot.data[4], kdl_rot.data[5],
         kdl_rot.data[6], kdl_rot.data[7], kdl_rot.data[8];
  output.block(0, 0, 3, 7) = rot * output.block(0, 0, 3, 7);
  output.block(3, 0, 3, 7) = rot * output.block(3, 0, 3, 7);
}

void Robot::getTaskPose(KDL::Frame &output, const int chain_index)
{
  static float temp[12];
  FRI->GetMeasuredCartPose(temp);
  KDL::Vector vec = KDL::Vector(temp[3], temp[7], temp[11]);
  KDL::Rotation rot = KDL::Rotation(temp[0], temp[1], temp[2], temp[4], temp[5], temp[6], temp[8], temp[9], temp[10]);
  output.p = vec;
  output.M = rot;
}
void Robot::getTaskPose(arma::mat &output, const int chain_index)
{
  getTaskPoseTemplate(output, chain_index);
}
void Robot::getTaskPose(Eigen::MatrixXd &output, const int chain_index)
{
  getTaskPoseTemplate(output, chain_index);
}

void Robot::getTaskPosition(KDL::Vector &output, const int chain_index)
{
  getTaskPositionTemplate(output, chain_index);
}
void Robot::getTaskPosition(arma::vec &output, const int chain_index)
{
  output.resize(3);
  getTaskPositionTemplate(output, chain_index);
}
void Robot::getTaskPosition(Eigen::Vector3d &output, const int chain_index)
{
  output.resize(3);
  getTaskPositionTemplate(output, chain_index);
}

void Robot::getTaskOrientation(KDL::Rotation &output, const int chain_index)
{
  getTaskOrientationTemplate(output, chain_index);
}
void Robot::getTaskOrientation(arma::mat &output, const int chain_index)
{
  output.resize(3, 3);
  getTaskOrientationTemplate(output, chain_index);
}
void Robot::getTaskOrientation(Eigen::Matrix3d &output, const int chain_index)
{
  output.resize(3, 3);
  getTaskOrientationTemplate(output, chain_index);
}

void Robot::getTwist(KDL::Twist &output, const int chain_index)
{
  getTwistTemplate(output, chain_index);
}
void Robot::getTwist(arma::vec &output, const int chain_index)
{
  output.resize(6);
  getTwistTemplate(output, chain_index);
}
void Robot::getTwist(Eigen::VectorXd &output, const int chain_index)
{
  output.resize(6);
  getTwistTemplate(output, chain_index);
}

void Robot::getExternalWrench(KDL::Wrench &output, const int chain_index)
{
  getExternalWrenchTemplate(output, chain_index);
}
void Robot::getExternalWrench(arma::vec &output, const int chain_index)
{
  output.resize(6);
  getExternalWrenchTemplate(output, chain_index);
}
void Robot::getExternalWrench(Eigen::VectorXd &output, const int chain_index)
{
  output.resize(6);
  getExternalWrenchTemplate(output, chain_index);
}

void Robot::setWrench(const KDL::Wrench &input, const int chain_index)
{
  setWrenchTemplate(input, chain_index);
}
void Robot::setWrench(const arma::vec &input, const int chain_index)
{
  setWrenchTemplate(input, chain_index);
}
void Robot::setWrench(const Eigen::VectorXd &input, const int chain_index)
{
  setWrenchTemplate(input, chain_index);
}

void Robot::startJointPositionController()
{
  // wait one tick
  FRI->WaitForKRCTick();
  std::cout << "[JointPosController::startController] Starting joint position control." << std::endl;
  int ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
  this->mode = arl::robot::Mode::POSITION_CONTROL;
  // if there s a problem
  if ((ResultValue != 0) && (ResultValue != EALREADY)) {
    std::cout << "[JointPosController::startController] "
              << "An error occurred during starting up the robot. I will stop controller." << std::endl;
    stopController();
    return;
  }
  std::cout << "[JointPosController::startController] " << "Finished" << std::endl;
}

void Robot::startJointTorqueController()
{
  // wait one tick
  FRI->WaitForKRCTick();
  // temp variables
  float stiffness[7];
  float stiffnessCart[6];
  float damping[7];
  float dampingCart[6];
  float torques[7];
  float q[7];
  // put zeros everywhere
  for (int i = 0; i < 7; i++) {
    stiffness[i] = 0;
    damping[i] = 0;
    torques[i] = 0;
  }
  for (int i = 0; i < 6; i++) {
    stiffnessCart[i] = 0;
    dampingCart[i] = 0;
  }
  // set stiffness to zero
  FRI->SetCommandedJointStiffness(stiffness);
  // set stiffness to zero
  FRI->SetCommandedCartStiffness(stiffnessCart);
  // set damping to zero
  FRI->SetCommandedJointDamping(damping);
  // set damping to zero
  FRI->SetCommandedCartDamping(dampingCart);
  // set additional torques to zero
  FRI->SetCommandedJointTorques(torques);
  // set commanded joint positions to current
  FRI->GetCommandedJointPositions(q);
  FRI->SetCommandedJointPositions(q);
  std::cout << "[KukaTorqueController::startController] Starting torque control." << std::endl;
  int ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);
  // if there s a problem
  this->mode = arl::robot::Mode::TORQUE_CONTROL;
  if ((ResultValue != 0) && (ResultValue != EALREADY))
  {
    std::cout << "[KukaTorqueController::startController] "
              << "An error occurred during starting up the robot. I will stop controller." << std::endl;
    stopController();
    return;
  }
  std::cout << "[KukaTorqueController::startController] " << "Finished" << std::endl;
}

void Robot::startCartImpController()
{
  // wait one tick
  FRI->WaitForKRCTick();
  // temp variables
  float stiffness[7];
  float stiffnessCart[6];
  float damping[7];
  float dampingCart[6];
  float torques[7];
  float p[12];
  // put zeros everywhere
  for (int i = 0; i < 7; i++) {
    stiffness[i] = 0;
    damping[i] = 0;
    torques[i] = 0;
  }
  for (int i = 0; i < 6; i++) {
    stiffnessCart[i] = 0;
    dampingCart[i] = 0;
  }
  // set stiffness to zero
  FRI->SetCommandedJointStiffness(stiffness);
  // set stiffness to zero
  FRI->SetCommandedCartStiffness(stiffnessCart);
  // set damping to zero
  FRI->SetCommandedJointDamping(damping);
  // set damping to zero
  FRI->SetCommandedCartDamping(dampingCart);
  // set additional torques to zero
  FRI->SetCommandedJointTorques(torques);
  // set commanded pose to current (mirror values)
  FRI->GetMeasuredCartPose(p);
  FRI->SetCommandedCartPose(p);
  std::cout << "[KukaCartImpedanceController::startController] Starting Cartesian Impedance control." << std::endl;
  int ResultValue = FRI->StartRobot(FastResearchInterface::CART_IMPEDANCE_CONTROL);
  // if there is a problem
  this->mode = arl::robot::Mode::IMPEDANCE_CONTROL;
  if ((ResultValue != 0) && (ResultValue != EALREADY))
  {
    std::cout << "[KukaCartImpedanceController::startController] "
              << "An error occurred during starting up the robot. I will stop controller." << std::endl;
    stopController();
    return;
  }
  std::cout << "[KukaCartImpedanceController::startController] " << "Finished" << std::endl;
}

void Robot::stopController()
{
  if (this->mode != arl::robot::Mode::STOPPED)
  {
    FRI->WaitForKRCTick();
    // printouts
    std::cout << "[KukaController::stopController] Stopping  control." << std::endl;

    float pose[12];
    float poseoff[12];

    float q[7];
    float qoff[7];
    float torques[7];

    // set commanded joint positions to current commanded
    FRI->GetCommandedJointPositions(q);
    FRI->GetCommandedJointPositionOffsets(qoff);

    for (int i = 0; i < 7; i++)
    {
      q[i] += qoff[i];
      torques[i] = 0.0;
    }

    FRI->SetCommandedJointPositions(q);

    // set commanded pose  to current commanded
    FRI->GetCommandedCartPose(pose);
    FRI->GetCommandedCartPoseOffsets(poseoff);
    for (int i = 0; i < 12; i++)
    {
      pose[i] += poseoff[i];
    }
    FRI->SetCommandedCartPose(pose);

    // set joint torques to zero
    FRI->SetCommandedJointTorques(torques);

    std::cout << "StopRobot...\n";
    // call stanford command
    FRI->StopRobot();
    this->mode = arl::robot::Mode::STOPPED;
    std::cout << "Done!\n";
  }
  // lower the flag
  std::cout << "[KukaController::stopController] " << "Finished" << std::endl;
}

void Robot::startLogging()
{
  printf("[KukaController::startLogging] Starting to write an output file...\n");
  int ResultValue = 0;
  ResultValue = FRI->PrepareLogging("Test");
  if (ResultValue == 0)
  {
    printf("[KukaController::startLogging] Logging successfully prepared.\n");
  }
  else
  {
    printf("[KukaController::startLogging] Error at FRI->PrepareLogging(): %s\n", strerror(ResultValue));
  }
  ResultValue = FRI->StartLogging();
  if (ResultValue == 0)
  {
    printf("[KukaController::startLogging] Logging successfully started.\n");
  }
  else
  {
    printf("[KukaController::startLogging] Error at FRI->StartLogging(): %s\n", strerror(ResultValue));
  }
}


void Robot::stopLogging()
{
  printf("[KukaController::stopLogging] Stopping to write an output file...\n");
  int ResultValue = 0;
  ResultValue = FRI->StopLogging();
  if (ResultValue == 0)
  {
    printf("[KukaController::stopLogging] Logging successfully stopped.\n");
  }
  else
  {
    printf("[KukaController::stopLogging] Error at FRI->StopLogging(): %s\n", strerror(ResultValue));
  }
  ResultValue = FRI->WriteLoggingDataFile();
  if (ResultValue == 0)
  {
    printf("[KukaController::stopLogging] Logging data file successfully written.\n");
  }
  else
  {
    printf("[KukaController::stopLogging] Error at FRI->WriteLoggingDataFile(): %s\n", strerror(ResultValue));
  }
}

void Robot::saveLastJointPosition(float input[7])
{
  for (size_t i = 0; i < 7; i++)
  {
    last_jnt_pos[i] = input[i];
  }
}

void Robot::saveLastJointPosition()
{
  static float temp[7];
  FRI->GetMeasuredJointPositions(temp);
  for (size_t i = 0; i < 7; i++)
  {
    last_jnt_pos[i] = temp[i];
  }
}
void Robot::stop()
{
  FRI->StopRobot();
}

bool Robot::isOk()
{
  static bool ok;
  ok = FRI->IsMachineOK();
  // if (!ok)
  // {
  //   ROS_ERROR_STREAM("[autharl_fri/lwr_robot] " << "FRI::IsMachineOK() returned false.");
  // }
  return ok;
}
}  // namespace robot
}  // namespace lwr
