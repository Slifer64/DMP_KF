#include <dmp_kf/Robot/Sim_Robot.h>
#include <math_lib/math_lib.h>

Sim_Robot::Sim_Robot()
{
  robot.reset(new as64_::RobotSim());

  N_JOINTS = robot->getNumJoints();
}

Sim_Robot::~Sim_Robot()
{

}

bool Sim_Robot::isOk()
{
  if (safety_check_on && getMode()==Robot::Mode::VELOCITY_CONTROL) safetyCheck();
  if (!robot->isOk()) err_msg = robot->getErrMsg();

  return (getSafetyStatus()==Robot::SafetyStatus::OK && robot->isOk());
}
void Sim_Robot::enable()
{
  setSafetyStatus(Robot::SafetyStatus::OK);
  robot->enable();
}

void Sim_Robot::init()
{
  vel_cmd = arma::vec().zeros(6);

  readParams();

  setSafetyStatus(Robot::SafetyStatus::OK);

  this->update();
}

double Sim_Robot::getControlCycle() const
{
  return robot->getCtrlCycle();
}

void Sim_Robot::update()
{
  robot->update();
}

void Sim_Robot::command()
{
  arma::mat J;
  arma::vec dq;

  switch (this->getMode())
  {
    case Robot::Mode::VELOCITY_CONTROL:
      J = robot->getJacobian();
      dq = arma::pinv(J)*vel_cmd;
      robot->setJointsVelocity(dq);
      break;
    case Robot::Mode::FREEDRIVE_MODE:
      // do nothing for robot_sim
      break;
    case Robot::Mode::IDLE_MODE:
      // do nothing for robot_sim
      break;
  }
}

void Sim_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == this->getMode()) return;

  switch (mode){
    case VELOCITY_CONTROL:
      robot->setMode(as64_::RobotSim::Mode::JOINT_VEL_CONTROL);
      PRINT_INFO_MSG("Robot set in VELOCITY_CONTROL MODE.\n");
      break;
    case FREEDRIVE_MODE:
      robot->setMode(as64_::RobotSim::Mode::FREEDRIVE);
      PRINT_INFO_MSG("Robot set in FREEDRIVE MODE.\n");
      break;
    case IDLE_MODE:
      robot->setMode(as64_::RobotSim::Mode::IDLE);
      PRINT_INFO_MSG("Robot set in IDLE MODE.\n");
      break;
  }
  this->mode = mode;
}

void Sim_Robot::stop()
{
  robot->setMode(as64_::RobotSim::Mode::IDLE);
}

void Sim_Robot::setTaskVelocity(const arma::vec &vel)
{
  if (this->getMode() != Robot::Mode::VELOCITY_CONTROL)
  {
    throw std::runtime_error("[ERROR] Sim_Robot::setTaskVelocity: Current mode is \"" + getModeName() + "\".\n");
  }

  vel_cmd = vel;
}

void Sim_Robot::setJointTrajectory(const arma::vec &qT, double duration)
{
  robot->setJointsTrajectory(qT, duration);
}

arma::vec Sim_Robot::getJointPosition() const
{
  return robot->getJointsPosition();
}

arma::mat Sim_Robot::getTaskPose() const
{
  return robot->getTaskPose();
}

arma::vec Sim_Robot::getTaskPosition() const
{
  return robot->getTaskPosition();
}

arma::vec Sim_Robot::getTaskOrientation() const
{
  return robot->getTaskOrientation();
}

arma::vec Sim_Robot::getTaskWrench()
{
  arma::vec Fext = robot->getExternalForce();

  arma::vec sign_Fext = arma::sign(Fext);
  arma::vec Fext2 = Fext - sign_Fext%Fext_dead_zone;
  Fext2 = 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);

  return Fext2;
}
