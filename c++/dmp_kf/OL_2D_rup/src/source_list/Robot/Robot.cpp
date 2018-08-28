#include <OL_2D_rup/Robot/Robot.h>
#include <ros/package.h>
#include <param_lib/param_lib.h>

Robot::Robot()
{
  modeName.resize(3);
  modeName[0] = "VELOCITY_CONTROL";
  modeName[1] = "FREEDRIVE_MODE";
  modeName[2] = "IDLE_MODE";

  vel_cmd = arma::vec().zeros(6);

  readParams();

  setSafetyStatus(Robot::SafetyStatus::OK);
}

Robot::~Robot()
{

}

void Robot::readParams(const char *params_file)
{
  std::string path_to_config_file;
  if (params_file != NULL) path_to_config_file = *params_file;
  else path_to_config_file = ros::package::getPath(PACKAGE_NAME)+ "/config/Robot_config.yml";
  as64_::param_::Parser parser(path_to_config_file);

  if (!parser.getParam("enable_safety_check", safety_check_on)) safety_check_on = false;
  if (!parser.getParam("vel_limit", vel_limit)) vel_limit = arma::vec().ones(6) * 100;
  if (!parser.getParam("wrench_limit", wrench_limit)) wrench_limit = arma::vec().ones(6) * 1000;
  if (!parser.getParam("Fext_dead_zone", Fext_dead_zone)) Fext_dead_zone = arma::vec().zeros(6);
}

Robot::Mode Robot::getMode() const
{
  return this->mode;
}

std::string Robot::getModeName() const
{
  return modeName[getMode()];
}

bool Robot::isOk()
{
  if (safety_check_on) safetyCheck();
  return getSafetyStatus()==Robot::SafetyStatus::OK;
}

void Robot::enable()
{
  setSafetyStatus(Robot::SafetyStatus::OK);
}

std::string Robot::getErrMsg() const
{
  return err_msg;
}

void Robot::safetyCheck()
{
  arma::vec Fext = this->getTaskWrench();

  setSafetyStatus(Robot::SafetyStatus::OK);
  err_msg = "";

  for (int i=0;i<Fext.size();i++)
  {
    if (std::fabs(Fext(i)) > wrench_limit(i))
    {
      setSafetyStatus(Robot::SafetyStatus::WRENCH_LIMIT);
      std::ostringstream out;
      out << "Wrench limit exceeded:\nFext(" << i+1 << ") = " << Fext(i) << "\n";
      err_msg = out.str();
      return;
    }
  }

  for (int i=0;i<vel_cmd.size();i++)
  {
    if (std::fabs(vel_cmd(i)) > vel_limit(i))
    {
      setSafetyStatus(Robot::SafetyStatus::VELOCITY_LIMIT);
      std::ostringstream out;
      out << "Velocity limit exceeded:\nV(" << i+1 << ") = " << vel_cmd(i) << "\n";
      err_msg = out.str();
      vel_cmd = vel_cmd*0;
      return;
    }
  }

}

void Robot::setSafetyStatus(const Robot::SafetyStatus &status)
{
  safe_status = status;
}

Robot::SafetyStatus Robot::getSafetyStatus()
{
  return safe_status;
}
