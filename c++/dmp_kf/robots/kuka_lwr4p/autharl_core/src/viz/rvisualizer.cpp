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

#include <autharl_core/viz/rvisualizer.h>
#include <autharl_core/math/orientation.h>
#include <autharl_core/utils/kdl_arma.h>
#include <geometry_msgs/Pose.h>

#include <string>

namespace arl
{
namespace viz
{
RVisualizer::RVisualizer() :
  RVisualizer(DEFAULT_BASE_FRAME, DEFAULT_RVIZ_TOPIC)
{
}

RVisualizer::RVisualizer(const std::string& base_frame,
                         const std::string& rviz_topic) :
  rviz_visual_tools::RvizVisualTools(base_frame, rviz_topic)
{
  // Clear messages
  this->deleteAllMarkers();
  this->enableBatchPublishing();

  vector_marker_.header.frame_id = this->getBaseFrame();
  vector_marker_.ns = "Vector";
  vector_marker_.id = 0;
  vector_marker_.type = visualization_msgs::Marker::ARROW;
  vector_marker_.action = visualization_msgs::Marker::ADD;
}

void RVisualizer::visualizeFrame(const arma::mat& pose, double length, double width, double alpha, double lifetime)
{
  geometry_msgs::Pose ros_pose;
  arma::vec quat = math::rotToQuat(pose.submat(0, 0, 2, 2));

  ros_pose.position.x = pose(0, 3);
  ros_pose.position.y = pose(1, 3);
  ros_pose.position.z = pose(2, 3);

  ros_pose.orientation.w = quat(0);
  ros_pose.orientation.x = quat(1);
  ros_pose.orientation.y = quat(2);
  ros_pose.orientation.z = quat(3);

  setAlpha(alpha);
  setLifetime(lifetime);
  this->publishAxis(ros_pose, length, width);
  this->triggerBatchPublish();
}

void RVisualizer::visualizeFrame(const KDL::Frame& pose, double length, double width, double alpha, double lifetime)
{
  geometry_msgs::Pose ros_pose;

  ros_pose.position.x = pose.p(0);
  ros_pose.position.y = pose.p(1);
  ros_pose.position.z = pose.p(2);

  arma::mat arma_rot;
  utils::kdlToArma(pose.M, &arma_rot);
  arma::vec quat = math::rotToQuat(arma_rot);
  ros_pose.orientation.w = quat(0);
  ros_pose.orientation.x = quat(1);
  ros_pose.orientation.y = quat(2);
  ros_pose.orientation.z = quat(3);

  setAlpha(alpha);
  setLifetime(lifetime);
  this->publishAxis(ros_pose, length, width);
  this->triggerBatchPublish();
}
void RVisualizer::visualizeText(const arma::mat& pose, const std::string&
    text, rviz_visual_tools::colors color, rviz_visual_tools::scales scale,
    double alpha, double lifetime)
{
  geometry_msgs::Pose ros_pose;
  arma::vec quat = math::rotToQuat(pose.submat(0, 0, 2, 2));

  ros_pose.position.x = pose(0, 3);
  ros_pose.position.y = pose(1, 3);
  ros_pose.position.z = pose(2, 3);

  ros_pose.orientation.w = quat(0);
  ros_pose.orientation.x = quat(1);
  ros_pose.orientation.y = quat(2);
  ros_pose.orientation.z = quat(3);

  setAlpha(alpha);
  setLifetime(lifetime);
  this->publishText(ros_pose, text, color, scale);
  this->triggerBatchPublish();
}

}  // namespace viz
}  // namespace arl
