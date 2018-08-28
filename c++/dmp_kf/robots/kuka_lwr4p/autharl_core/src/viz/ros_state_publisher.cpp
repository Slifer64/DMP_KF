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

#include <autharl_core/viz/ros_state_publisher.h>
#include <kdl/frames_io.hpp>

namespace arl
{
namespace viz
{
RosStatePublisher::RosStatePublisher(std::shared_ptr<robot::Robot> r, double rate,
                                     const std::string& frame, const std::string& topic) :
  Controller(r),
  loop_rate(rate),
  msr_jnt_pos(robot->model->getNrOfJoints()),
  rviz(frame, topic)
{
  joint_state_pub = n.advertise<sensor_msgs::JointState>("/autharl_joint_state", 1);
  joint_state.name.resize(robot->model->getNrOfJoints());
  joint_state.position.resize(robot->model->getNrOfJoints());
  this->frame.push_back(arma::mat(4, 4));
}

void RosStatePublisher::measure()
{
  robot->getJointPosition(msr_jnt_pos, -1);
}

void RosStatePublisher::command()
{
  joint_state_pub.publish(joint_state);
  for (int i = 0; i < frame.size(); i++)
  {
    rviz.visualizeFrame(frame.at(i), 0.1, 0.01, 1, loop_rate.expectedCycleTime().toSec());
  }
}

void RosStatePublisher::update()
{
  joint_state.header.stamp = ros::Time::now();
  for (int i = 0; i < robot->model->getNrOfJoints(); i++)
  {
    joint_state.name.at(i) = robot->model->getJointName(i);
    joint_state.position.at(i) = msr_jnt_pos(i);
  }
}

bool RosStatePublisher::run()
{
  while (!stop())
  {
    measure();
    update();
    command();
    loop_rate.sleep();
  }
  return true;
}

void RosStatePublisher::setFrame(const std::vector<arma::mat>& f)
{
  this->frame = f;
}

bool RosStatePublisher::stop()
{
  return !ros::ok();
}

}  // namespace viz
}  // namespace arl
