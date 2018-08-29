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

#include <autharl_core/robot/state.h>

namespace arl
{
namespace robot
{
State::State(const int& num_joints)
{
  msr.jnt.pos = KDL::JntArray(num_joints);
  msr.jnt.vel = KDL::JntArray(num_joints);
  msr.jnt.eff = KDL::JntArray(num_joints);
  msr.jnt.ext_eff = KDL::JntArray(num_joints);
  for (int i = 0; i < num_joints; i++)
  {
    msr.jnt.jac.push_back(KDL::Jacobian(i + 1));
    msr.jnt.pose.push_back(KDL::Frame());
  }

  msr.task.pose = KDL::Frame();
  msr.task.vel = KDL::Twist();
  msr.task.eff = KDL::Wrench(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0));
  msr.task.ext_eff = KDL::Wrench(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0));
  msr.task.jac = KDL::Jacobian(num_joints);


  cmd.jnt.pos = KDL::JntArray(num_joints);
  cmd.jnt.vel = KDL::JntArray(num_joints);
  cmd.jnt.eff = KDL::JntArray(num_joints);
  cmd.jnt.ext_eff = KDL::JntArray(num_joints);
  for (int i = 0; i < num_joints; i++)
  {
    cmd.jnt.jac.push_back(KDL::Jacobian(i + 1));
  }

  cmd.task.pose = KDL::Frame();
  cmd.task.vel = KDL::Twist();
  cmd.task.eff = KDL::Wrench(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0));
  cmd.task.ext_eff = KDL::Wrench(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0));
  cmd.task.jac = KDL::Jacobian(num_joints);
}

std::ostream& operator<<(std::ostream &os, const State &state)
{
  os << "Measured Robot State:" << std::endl;
  os << "Joint Positions: " << state.msr.jnt.pos << std::endl;
  os << "Joint Velocities: " << state.msr.jnt.vel << std::endl;
  os << "Joint Torques: " << state.msr.jnt.eff << std::endl;
  os << "External Joint Torques: " << state.msr.jnt.ext_eff << std::endl;
  os << "Tool Pose (Position/Orientation): " << state.msr.task.pose.p <<
    std::endl << state.msr.task.pose.M << std::endl;
  os << "Tool Twist (Linear/Angular velocity): " << state.msr.task.vel.vel <<
    state.msr.task.vel.rot << std::endl;
  os << "Tool Wrench (Force/Torque): " << state.msr.task.eff.force <<
    state.msr.task.eff.torque << std::endl;
  os << "Jacobian: " << std::endl;
  os << state.msr.task.jac << std::endl;
}
}  // namespace robot
}  // namespace arl
