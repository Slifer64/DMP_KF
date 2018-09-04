#include <dmp_kf/Robot/UR10_Robot.h>

UR10_Robot::UR10_Robot()
{
  robot.reset(new ur10_::Robot());
}

UR10_Robot::~UR10_Robot()
{

}

bool UR10_Robot::isOk()
{
  return robot->isOk();
}

void UR10_Robot::init()
{
  vel_cmd = arma::vec().zeros(6);

  readParams();

  setSafetyStatus(Robot::SafetyStatus::OK);

  this->update();
}

double UR10_Robot::getControlCycle() const
{
  return robot->getControlCycle();
}

void UR10_Robot::update()
{
  robot->waitNextCycle();
}

void UR10_Robot::command()
{
  switch (this->getMode())
  {
    case Robot::Mode::VELOCITY_CONTROL:
      robot->setTaskVelocity(vel_cmd);
      break;
    case Robot::Mode::FREEDRIVE_MODE:
      // do nothing for ur10
      break;
    case Robot::Mode::IDLE_MODE:
      robot->sleep(getControlCycle());
      // do nothing for ur10
      break;
  }
}

void UR10_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == this->getMode()) return;

  switch (mode){
    case VELOCITY_CONTROL:
      robot->setMode(ur10_::Mode::VELOCITY_CONTROL);
      PRINT_INFO_MSG("Robot set in VELOCITY_CONTROL MODE.\n");
      break;
    case FREEDRIVE_MODE:
      robot->setMode(ur10_::Mode::FREEDRIVE_MODE);
      PRINT_INFO_MSG("Robot set in FREEDRIVE MODE.\n");
      break;
    case IDLE_MODE:
      robot->setMode(ur10_::Mode::POSITION_CONTROL);
      PRINT_INFO_MSG("Robot set in IDLE MODE.\n");
      break;
  }
  this->mode = mode;
}

void UR10_Robot::stop()
{
  robot->stopj(5.0);
}

void UR10_Robot::setTaskVelocity(const arma::vec &vel)
{
  if (this->getMode() != Robot::Mode::VELOCITY_CONTROL)
  {
    throw std::runtime_error("[ERROR] UR10_Robot::setTaskVelocity: Current mode is \"" + getModeName() + "\".\n");
  }
  vel_cmd = vel;
}

void UR10_Robot::setJointTrajectory(const arma::vec &qT, double duration)
{
  robot->setJointTrajectory(qT, duration);
}

arma::vec UR10_Robot::getJointPosition() const
{
  return robot->getJointPosition();
}

arma::mat UR10_Robot::getTaskPose() const
{
  return robot->getTaskPose();
}

arma::vec UR10_Robot::getTaskPosition() const
{
  return robot->getTaskPosition();
}

arma::vec UR10_Robot::getTaskOrientation() const
{
  return robot->getTaskOrientation();
}

arma::vec UR10_Robot::getTaskWrench() const
{
  arma::vec Fext = robot->getTaskWrench();
  arma::vec sign_Fext = arma::sign(Fext);
  arma::vec Fext2 = Fext - sign_Fext%Fext_dead_zone;
  Fext2 = 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);

  return Fext2;
}
