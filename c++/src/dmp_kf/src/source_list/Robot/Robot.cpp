#include <dmp_kf/Robot/Robot.h>
#include <ros/package.h>
#include <io_lib/parser.h>

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
  as64_::io_::Parser parser(path_to_config_file);

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

Eigen::Vector4d Robot::rotm2quat(Eigen::Matrix3d rotm) const
{
    Eigen::Quaternion<double> temp_quat(rotm);
    Eigen::Vector4d quat;
    quat << temp_quat.w(), temp_quat.x(), temp_quat.y(), temp_quat.z();

    quat = quat * (2*(quat(0)>=0)-1); // to avoid discontinuities

    return quat;
}

arma::vec Robot::rotm2quat(const arma::mat &rotm) const
{
  arma::vec quat(4);

  Eigen::Map<const Eigen::Matrix3d> rotm_wrapper(rotm.memptr());
  Eigen::Map<Eigen::Vector4d> quat_wrapper(quat.memptr());
  quat_wrapper = rotm2quat(rotm_wrapper);

  return quat;
}
