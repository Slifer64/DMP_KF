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

#include <autharl_core/robot/robot_sim.h>

namespace arl
{
namespace robot
{
RobotSim::RobotSim(std::shared_ptr<arl::robot::Model> m, double cycle_rate_sec) :
  Robot(m, "Simulated Robot"),
  loop_rate(1/cycle_rate_sec)
{
  for (int i = 0; i < model->getNrOfChains(); i++)
  {
    state.push_back(arl::robot::State(model->getNrOfJoints(i)));
    last_jnt_pos.push_back(KDL::JntArray(model->getNrOfJoints(i)));
  }
  cycle = cycle_rate_sec;
}

void RobotSim::setMode(arl::robot::Mode mode, int chain_index)
{
  switch (mode)
  {
    case arl::robot::Mode::STOPPED :
      this->mode = arl::robot::Mode::STOPPED;
      break;
    case arl::robot::Mode::POSITION_CONTROL :
      getJointPosition(joint_pos_prev, -1);
      this->mode = arl::robot::Mode::POSITION_CONTROL;
      break;
    case arl::robot::Mode::VELOCITY_CONTROL :
      getJointPosition(joint_pos_prev, -1);
      this->mode = arl::robot::Mode::VELOCITY_CONTROL;
      break;
    default: std::cout << "Mode " << mode << " Not available" << std::endl;
  }
}

void RobotSim::setJointTrajectory(const KDL::JntArray &input, double duration, const int chain_index)
{
  setJointTrajectoryTemplate(input, duration, chain_index);
}
void RobotSim::setJointTrajectory(const arma::vec &input, double duration, const int chain_index)
{
  setJointTrajectoryTemplate(input, duration, chain_index);
}
void RobotSim::setJointTrajectory(const Eigen::VectorXd &input, double duration, const int chain_index)
{
  setJointTrajectoryTemplate(input, duration, chain_index);
}

void RobotSim::setJointPosition(const KDL::JntArray &input, const int chain_index)
{
  setJointPositionTemplate(input, chain_index);
}
void RobotSim::setJointPosition(const arma::vec &input, const int chain_index)
{
  setJointPositionTemplate(input, chain_index);
}
void RobotSim::setJointPosition(const Eigen::VectorXd &input, const int chain_index)
{
  setJointPositionTemplate(input, chain_index);
}

void RobotSim::getJointPosition(KDL::JntArray &output, const int chain_index)
{
  getJointPositionTemplate(output, chain_index);
}
void RobotSim::getJointPosition(arma::vec &output, const int chain_index)
{
  getJointPositionTemplate(output, chain_index);
}
void RobotSim::getJointPosition(Eigen::VectorXd &output, const int chain_index)
{
  getJointPositionTemplate(output, chain_index);
}

void RobotSim::getJacobian(KDL::Jacobian &output, const int chain_index)
{
  model->jac_solver.at(chain_index).JntToJac(state.at(chain_index).msr.jnt.pos, output);
}
void RobotSim::getJacobian(arma::mat &output, const int chain_index)
{
  output.resize(6, model->getNrOfJoints(chain_index));
  getJacobianTemplate(output, chain_index);
}
void RobotSim::getJacobian(Eigen::MatrixXd &output, const int chain_index)
{
  output.resize(6, model->getNrOfJoints(chain_index));
  getJacobianTemplate(output, chain_index);
}

void RobotSim::getTaskPose(KDL::Frame &output, const int chain_index)
{
  model->fk_solver.at(chain_index).JntToCart(state.at(chain_index).msr.jnt.pos, output);
}
void RobotSim::getTaskPose(arma::mat &output, const int chain_index)
{
  output.resize(4, 4);
  getTaskPoseTemplate(output, chain_index);
  output(3, 0) = 0;
  output(3, 1) = 0;
  output(3, 2) = 0;
  output(3, 3) = 1;
}
void RobotSim::getTaskPose(Eigen::MatrixXd &output, const int chain_index)
{
  output.resize(4, 4);
  getTaskPoseTemplate(output, chain_index);
  output(3, 0) = 0;
  output(3, 1) = 0;
  output(3, 2) = 0;
  output(3, 3) = 1;
}

void RobotSim::getTaskPosition(KDL::Vector &output, const int chain_index)
{
  KDL::Frame pose;
  model->fk_solver.at(chain_index).JntToCart(state.at(chain_index).msr.jnt.pos, pose);
  output = pose.p;
}
void RobotSim::getTaskPosition(arma::vec &output, const int chain_index)
{
  output.resize(3);
  getTaskPositionTemplate(output, chain_index);
}
void RobotSim::getTaskPosition(Eigen::Vector3d &output, const int chain_index)
{
  output.resize(3);
  getTaskPositionTemplate(output, chain_index);
}

void RobotSim::getTaskOrientation(KDL::Rotation &output, const int chain_index)
{
  KDL::Frame pose;
  model->fk_solver.at(chain_index).JntToCart(state.at(chain_index).msr.jnt.pos, pose);
  output = pose.M;
}
void RobotSim::getTaskOrientation(arma::mat &output, const int chain_index)
{
  output.resize(3, 3);
  getTaskOrientationTemplate(output, chain_index);
}
void RobotSim::getTaskOrientation(Eigen::Matrix3d &output, const int chain_index)
{
  output.resize(3, 3);
  getTaskOrientationTemplate(output, chain_index);
}

void RobotSim::getTwist(KDL::Twist &output, const int chain_index)
{
  getTwistTemplate(output, chain_index);
}
void RobotSim::getTwist(arma::vec &output, const int chain_index)
{
  output.resize(6);
  getTwistTemplate(output, chain_index);
}
void RobotSim::getTwist(Eigen::VectorXd &output, const int chain_index)
{
  output.resize(6);
  getTwistTemplate(output, chain_index);
}

bool RobotSim::isOk()
{
  return ros::ok();
}

void RobotSim::waitNextCycle()
{
  loop_rate.sleep();
}
}  // namespace robot
}  // namespace arl
