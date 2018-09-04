/**
 * Copyright (C) 2016 AUTH-lwr
 */

#include <lwr4p/Robot.h>
#include <iostream>
#include <string>

namespace lwr4p
{

Robot::Robot(const char *path_to_FRI_init): N_JOINTS(7)
{
  if (path_to_FRI_init == NULL)
    FRI.reset(new FastResearchInterface("/home/user/lwr/980500-FRI-Driver.init"));
  else
    FRI.reset(new FastResearchInterface(path_to_FRI_init));

  cycle = FRI->GetFRICycleTime();
  this->mode = lwr4p::Mode::UNDEFINED;  // Dummy initialization before stopping controller
  stopController();  // Initially the robot is stopped
  startJointPositionController();
}

void Robot::waitNextCycle()
{
  FRI->WaitForKRCTick();
}

void Robot::setMode(lwr4p::Mode mode)
{
  switch (mode)
  {
    case lwr4p::Mode::STOPPED :
      if (this->mode != lwr4p::Mode::STOPPED)
      {
        stopController();
      }
      saveLastJointPosition();
      break;
    case lwr4p::Mode::POSITION_CONTROL :
      if (this->mode != lwr4p::Mode::POSITION_CONTROL)
      {
        if (this->mode != lwr4p::Mode::STOPPED)
        {
          stopController();
        }
        startJointPositionController();
        saveLastJointPosition();
      }
      break;
    case lwr4p::Mode::VELOCITY_CONTROL :
      if (this->mode != lwr4p::Mode::VELOCITY_CONTROL)
      {
        if (this->mode != lwr4p::Mode::POSITION_CONTROL)
        {
          stopController();
          startJointPositionController();
        }
        this->mode = lwr4p::Mode::VELOCITY_CONTROL;
        saveLastJointPosition();
      }
      break;
    case lwr4p::Mode::TORQUE_CONTROL :
      if (this->mode != lwr4p::Mode::TORQUE_CONTROL)
      {
        if (this->mode != lwr4p::Mode::STOPPED)
        {
          stopController();
        }
        startJointTorqueController();
        saveLastJointPosition();
      }
      break;
    case lwr4p::Mode::IMPEDANCE_CONTROL :
      if (this->mode != lwr4p::Mode::IMPEDANCE_CONTROL)
      {
        if (this->mode != lwr4p::Mode::STOPPED)
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

arma::vec Robot::getJointPosition()
{
  arma::vec output(N_JOINTS);
  float temp[N_JOINTS];
  FRI->GetMeasuredJointPositions(temp);
  for (size_t i = 0; i < N_JOINTS; i++) {
    output(i) = temp[i];
  }
  return output;
}

arma::vec Robot::getJointTorque()
{
  arma::vec output(N_JOINTS);
  float joint_torques[N_JOINTS];
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

arma::vec Robot::getJointExternalTorque()
{
  arma::vec output(N_JOINTS);
  float estimated_external_joint_torques[N_JOINTS];
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

arma::mat Robot::getJacobian()
{
  arma::vec output = getJacobian();
  float temp[12];
  FRI->GetMeasuredCartPose(temp);
  arma::mat rot;
  rot << temp[0] << temp[1] << temp[2] << arma::endr
      << temp[4] << temp[5] << temp[6] << arma::endr
      << temp[8] << temp[9] << temp[10];
  output.submat(0, 0, 2, 6) = rot * output.submat(0, 0, 2, 6);
  output.submat(3, 0, 5, 6) = rot * output.submat(3, 0, 5, 6);

  return output;
}

arma::mat Robot::getTaskPose()
{
  arma::mat output = arma::mat().eye(4, 4);
  float temp[12];
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

arma::vec Robot::getTaskPosition()
{
  arma::vec output(3);
  float temp[12];
  FRI->GetMeasuredCartPose(temp);
  output(0) = temp[3];
  output(1) = temp[N_JOINTS];
  output(2) = temp[11];

  return output;
}

arma::mat Robot::getTaskOrientation()
{
  arma::mat output(3,3);
  float temp[12];
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

arma::vec Robot::getExternalWrench()
{
  arma::vec output(6);
  float estimated_external_cart_forces_and_torques[6];
  FRI->GetEstimatedExternalCartForcesAndTorques(estimated_external_cart_forces_and_torques);
  output(0) = estimated_external_cart_forces_and_torques[0];
  output(1) = estimated_external_cart_forces_and_torques[1];
  output(2) = estimated_external_cart_forces_and_torques[2];
  output(3) = estimated_external_cart_forces_and_torques[3];
  output(4) = estimated_external_cart_forces_and_torques[4];
  output(5) = estimated_external_cart_forces_and_torques[5];

  return output;
}

arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
{
  arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

  if (t < 0)
  {
    // before start
    retTemp.col(0) = p0;
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp.col(0) = pT;
  }
  else
  {
    // somewhere betweeen ...
    // position
    retTemp.col(0) = p0 +
                     (pT - p0) * (10 * pow(t / totalTime, 3) -
                     15 * pow(t / totalTime, 4) +
                     6 * pow(t / totalTime, 5));
    // vecolity
    retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) -
                     60 * pow(t, 3) / pow(totalTime, 4) +
                     30 * pow(t, 4) / pow(totalTime, 5));
    // acceleration
    retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) -
                     180 * pow(t, 2) / pow(totalTime, 4) +
                     120 * pow(t, 3) / pow(totalTime, 5));
  }

  // return vector
  return retTemp;
}

void Robot::setJointTrajectory(const arma::vec &input, double duration)
{
  // setJntPosTrajTemplate(input, duration)
  // inital joint position values
  arma::vec q0 = arma::zeros<arma::vec>(N_JOINTS);
  arma::vec temp = arma::zeros<arma::vec>(N_JOINTS);
  for (int i = 0; i < N_JOINTS; i++) {
    temp(i) = input(i);
  }
  q0 = getJointPosition();
  // keep last known robot mode
  lwr4p::Mode prev_mode = mode;
  arma::vec qref = q0;
  // start conttroller
  setMode(lwr4p::Mode::POSITION_CONTROL);
  // initalize time
  double t = 0.0;
  // the main while
  while (t < duration)
  {
    // waits for the next tick also
    FRI->WaitForKRCTick();
    // compute time now
    t += cycle;
    // update trajectory
    qref = get5thOrder(t, q0, temp, duration).col(0);
    // set joint positions
    setJointPosition(qref);
  }
  // reset last known robot mode
  setMode(prev_mode);
}

void Robot::setJointPosition(const arma::vec &input)
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
    float temp[N_JOINTS];
    // put the values from arma to float[]
    for (int i = 0; i < N_JOINTS; i++) {
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

void Robot::setJointVelocity(const arma::vec &input)
{
  if (this->mode == lwr4p::Mode::VELOCITY_CONTROL)
  {
    float temp[N_JOINTS];
    for (size_t i = 0; i < N_JOINTS; i++)
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

void Robot::setJointTorque(const arma::vec &input)
{
  if (this->mode == lwr4p::Mode::TORQUE_CONTROL)
  {
    float torques[N_JOINTS];
    for (size_t i = 0; i < N_JOINTS; i++)
    {
      torques[i] = input(i);
    }
    FRI->SetCommandedJointTorques(torques);
    // Mirror the joint positions and the cartesian pose in order to avoid
    // cartesian deviation errors
    float temp_position[N_JOINTS];
    FRI->GetMeasuredJointPositions(temp_position);
    FRI->SetCommandedJointPositions(temp_position);
    float temp_pose[12];
    FRI->GetMeasuredCartPose(temp_pose);
    FRI->SetCommandedCartPose(temp_pose);

    saveLastJointPosition(temp_position);
  }
  else
  {
    std::cerr << "setJointTorque only available in TORQUE_CONTROL mode" << std::endl;
  }
}

void Robot::setTaskPose(const arma::mat &input)
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
    float temp[12];
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

void Robot::setCartStiffness(const arma::vec &input)
{
  if (this->mode == lwr4p::Mode::IMPEDANCE_CONTROL)
  {
    if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
    {
      printf("[CartImpedanceController::setCartStiffness] Cartesian wrench is commanded with closed controller.\n");
      printf("Opening controller ...\n");
      // start te controller
      startCartImpController();
      // wait one tick
      FRI->WaitForKRCTick();
    }
    // temp variables
    float temp[6];
    // put the values from arma to float[]
    for (int i = 0; i < 6; i++) {
      temp[i] = input(i);
    }

    // set value
    FRI->SetCommandedCartStiffness(temp);
  }
  else
  {
    std::cerr << "setCartStiffness only available in IMPEDANCE_CONTROL mode" << std::endl;
  }
}

void Robot::setCartDamping(const arma::vec &input)
{
  if (this->mode == lwr4p::Mode::IMPEDANCE_CONTROL)
  {
    if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
    {
      printf("[CartImpedanceController::setCartDamping] Cartesian wrench is commanded with closed controller.\n");
      printf("Opening controller ...\n");
      // start te controller
      startCartImpController();
      // wait one tick
      FRI->WaitForKRCTick();
    }
    // temp variables
    float temp[6];
    // put the values from arma to float[]
    for (int i = 0; i < 6; i++) {
      temp[i] = input(i);
    }

    // set value
    FRI->SetCommandedCartDamping(temp);
  }
  else
  {
    std::cerr << "setCartDamping only available in IMPEDANCE_CONTROL mode" << std::endl;
  }
}

void Robot::setWrench(const arma::vec &input)
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
    float temp[6];
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

void Robot::startJointPositionController()
{
  // wait one tick
  FRI->WaitForKRCTick();
  std::cout << "[JointPosController::startController] Starting joint position control." << std::endl;
  int ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
  this->mode = lwr4p::Mode::POSITION_CONTROL;
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
  float stiffness[N_JOINTS];
  float stiffnessCart[6];
  float damping[N_JOINTS];
  float dampingCart[6];
  float torques[N_JOINTS];
  float q[N_JOINTS];
  // put zeros everywhere
  for (int i = 0; i < N_JOINTS; i++) {
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
  this->mode = lwr4p::Mode::TORQUE_CONTROL;
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
  float stiffness[N_JOINTS];
  float stiffnessCart[6];
  float damping[N_JOINTS];
  float dampingCart[6];
  float torques[N_JOINTS];
  float p[12];
  // put zeros everywhere
  for (int i = 0; i < N_JOINTS; i++) {
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
  this->mode = lwr4p::Mode::IMPEDANCE_CONTROL;
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
  if (this->mode != lwr4p::Mode::STOPPED)
  {
    FRI->WaitForKRCTick();
    // printouts
    std::cout << "[KukaController::stopController] Stopping  control." << std::endl;

    static float pose[12];
    static float poseoff[12];

    float q[N_JOINTS];
    float qoff[N_JOINTS];
    float torques[N_JOINTS];

    // set commanded joint positions to current commanded
    FRI->GetCommandedJointPositions(q);
    FRI->GetCommandedJointPositionOffsets(qoff);

    for (int i = 0; i < N_JOINTS; i++)
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
    this->mode = lwr4p::Mode::STOPPED;
    std::cout << "Done!\n";
  }
  // lower the flag
  std::cout << "[KukaController::stopController] " << "Finished" << std::endl;
}

void Robot::saveLastJointPosition(float input[7])
{
  for (size_t i = 0; i < N_JOINTS; i++)
  {
    last_jnt_pos[i] = input[i];
  }
}

void Robot::saveLastJointPosition()
{
  float temp[N_JOINTS];
  FRI->GetMeasuredJointPositions(temp);
  for (size_t i = 0; i < N_JOINTS; i++)
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
  //   ROS_ERROR_STREAM("[authlwr_fri/lwr_robot] " << "FRI::IsMachineOK() returned false.");
  // }
  return ok;
}

}  // namespace lwr
