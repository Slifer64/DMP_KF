/*******************************************************************************
 * Copyright (c) 2016-2017 Automation and Robotics Lab, AUTh
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

#include <autharl_core/robot/robot.h>
#include <autharl_core/utils/string.h>
#include <string>

namespace arl
{
namespace robot
{
Robot::Robot()
{
}

Robot::Robot(std::shared_ptr<Model> m, const std::string& name)
{
  robot_name = name;
  model = m;
  for (size_t i = 0; i < model->getNrOfChains(); i++)
  {
    state.push_back(State(model->getNrOfJoints(i)));
  }

  utils::getAlias(robot_name, &alias);
  joint_pos_prev.resize(model->getNrOfJoints());
}

void Robot::setJointPosition(const KDL::JntArray &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setJointPosition(const arma::vec &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setJointPosition(const Eigen::VectorXd &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::setJointTrajectory(const KDL::JntArray &input, double duration, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setJointTrajectory(const arma::vec &input, double duration, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setJointTrajectory(const Eigen::VectorXd &input, double duration, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::setJointVelocity(const KDL::JntArray &input, const int chain_index)
{
  setJointVelocityGeneric(input, chain_index);
}
void Robot::setJointVelocity(const arma::vec &input, const int chain_index)
{
  setJointVelocityGeneric(input, chain_index);
}
void Robot::setJointVelocity(const Eigen::VectorXd &input, const int chain_index)
{
  setJointVelocityGeneric(input, chain_index);
}

void Robot::setJointTorque(const KDL::JntArray &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setJointTorque(const arma::vec &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setJointTorque(const Eigen::VectorXd &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::setTaskPose(const KDL::Frame &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setTaskPose(const arma::mat &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setTaskPose(const Eigen::MatrixXd &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::setCartStiffness(const KDL::Wrench &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setCartStiffness(const arma::vec &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setCartStiffness(const Eigen::VectorXd &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::setCartDamping(const KDL::Wrench &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setCartDamping(const arma::vec &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setCartDamping(const Eigen::VectorXd &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::setTwist(const KDL::Twist &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setTwist(const arma::vec &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setTwist(const Eigen::VectorXd &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::setWrench(const KDL::Wrench &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setWrench(const arma::vec &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setWrench(const Eigen::VectorXd &input, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getJointPosition(KDL::JntArray &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJointPosition(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJointPosition(Eigen::VectorXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getJointVelocity(KDL::JntArray &output, const int chain_index)
{
  getJointVelocityGeneric(output, chain_index);
}
void Robot::getJointVelocity(arma::vec &output, const int chain_index)
{
  getJointVelocityGeneric(output, chain_index);
}
void Robot::getJointVelocity(Eigen::VectorXd &output, const int chain_index)
{
  getJointVelocityGeneric(output, chain_index);
}

void Robot::getJointTorque(KDL::JntArray &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJointTorque(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJointTorque(Eigen::VectorXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getJointExternalTorque(KDL::JntArray &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJointExternalTorque(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJointExternalTorque(Eigen::VectorXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getJointStiffness(KDL::JntArray &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJointStiffness(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJointStiffness(Eigen::VectorXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getJointDamping(KDL::JntArray &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJointDamping(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJointDamping(Eigen::VectorXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getTaskPose(KDL::Frame &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTaskPose(arma::mat &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTaskPose(Eigen::MatrixXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getTaskPosition(KDL::Vector &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTaskPosition(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTaskPosition(Eigen::Vector3d &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getTaskOrientation(KDL::Rotation &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTaskOrientation(arma::mat &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTaskOrientation(Eigen::Matrix3d &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getTwist(KDL::Twist &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTwist(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTwist(Eigen::VectorXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getWrench(KDL::Wrench &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getWrench(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getWrench(Eigen::VectorXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getExternalWrench(KDL::Wrench &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getExternalWrench(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getExternalWrench(Eigen::VectorXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getTaskStiffness(KDL::JntArray &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTaskStiffness(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTaskStiffness(Eigen::VectorXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getTaskDamping(KDL::JntArray &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTaskDamping(arma::vec &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getTaskDamping(Eigen::VectorXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::getJacobian(KDL::Jacobian &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJacobian(arma::mat &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::getJacobian(Eigen::MatrixXd &output, const int chain_index)
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::stop()
{
  throwFunctionIsNotImplemented(__func__);
}
void Robot::setMode(Mode mode, int chain_index)
{
  switch (mode)
  {
    case Mode::UNDEFINED :
      this->mode = Mode::UNDEFINED;
      break;
    case Mode::STOPPED :
      this->mode = Mode::STOPPED;
      break;
    case Mode::POSITION_CONTROL :
      this->mode = Mode::POSITION_CONTROL;
      break;
    case Mode::VELOCITY_CONTROL :
      getJointPosition(joint_pos_prev, -1);
      this->mode = Mode::VELOCITY_CONTROL;
      break;
    case Mode::TORQUE_CONTROL :
      this->mode = Mode::TORQUE_CONTROL;
      break;
    case Mode::IMPEDANCE_CONTROL :
      this->mode = Mode::IMPEDANCE_CONTROL;
      break;
    case Mode::JOINT_TRAJECTORY :
      this->mode = Mode::JOINT_TRAJECTORY;
      break;
    default: std::cout << "Mode " << mode << " Not available" << std::endl;
  }
}
void Robot::measure()
{
  throwFunctionIsNotImplemented(__func__);
}

void Robot::throwFunctionIsNotImplemented(std::string function_name)
{
  std::cerr << "[autharl::Robot] Error: The function " << function_name
            << " is not implemented by the used interface for the robot: "
            << robot_name << std::endl;
}

}  // namespace robot
}  // namespace arl
