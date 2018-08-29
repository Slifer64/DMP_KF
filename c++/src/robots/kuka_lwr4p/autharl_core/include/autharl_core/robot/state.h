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

#ifndef AUTHARL_CORE_ROBOT_STATE_H
#define AUTHARL_CORE_ROBOT_STATE_H

#include <vector>

#include <kdl/jntarray.hpp>  // KDL::JntArray
#include <kdl/frames.hpp>  // KDL::Frame, KDL::Vector, KDL::Wrench
#include <kdl/stiffness.hpp>  // KDL::Stiffness

#include <ostream>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

namespace arl
{
namespace robot
{
/**
 * The Robot State of the robot.
 */
class State
{
public:
  State();
  explicit State(const int&);
  // friend std::ostream& operator<< (std::ostream &os, const RobotState &state);

  /**
   * Data in the joint space of the robot
   */
  struct Joint
  {
    KDL::JntArray pos;  /**< Joint position in rads */
    std::vector<KDL::Frame> pose; /**< Cartesian pose (position/orientation) of each joint wrt the base frame */
    KDL::JntArray vel;  /**< Joint velocities in rads/sec */
    KDL::JntArray eff;  /**< Joint efford (i.e. joint torques) in N*m */
    KDL::JntArray ext_eff;  /**< External joint efford (i.e. joint torques) in N*m */

    KDL::JntArray stiff;  /**< Cartesian stiffness of the manipulator */
    KDL::JntArray damp;  /**< Cartesian stiffness of the manipulator */
    std::vector<KDL::Jacobian> jac;  /**< The Jacobian metrics for each joint of each kinematic chain */
  };

  /**
   * Data in the task (cartesian) space of the robot
   */
  struct Task
  {
    KDL::Frame pose;  /**< Cartesian pose (position/orientation) of tool frame wrt the base frame */
    KDL::Twist vel;  /**< Cartesian and rotational velocity (twist) of tool frame wrt the base frame */
    KDL::Wrench eff;  /**< Cartesian forces and torques (wrench) on the tool frame */
    KDL::Wrench ext_eff;  /**< External forces and torques (wrench) on the tool frame */

    KDL::Stiffness stiff;  /**< Cartesian stiffness of the manipulator */
    KDL::Stiffness damp;  /**< Cartesian damping of the manipulator */
    KDL::Jacobian jac;  /**< The Jacobian metrics for each kinematic chain */
  };

  /**
   * The defined system containing the joint space the task space and the Jacobians
   */
  struct System
  {
    Joint jnt;  /**< Data in the joint space of the robot */
    Task task;  /**< Data in the task (cartesian) space of the robot */
  };


  System msr;  /**< The measured by the sensors state */
  System cmd;  /**< The commanded state by the controllers */
};

std::ostream& operator<<(std::ostream &os, const State &state);
}  // namespace robot
}  // namespace arl

#endif  // AUTHARL_CORE_ROBOT_STATE_H
