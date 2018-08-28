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

#ifndef AUTHARL_CORE_VIZ_ROS_STATE_PUBLISHER_H
#define AUTHARL_CORE_VIZ_ROS_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <autharl_core/robot/controller.h>
#include <autharl_core/viz/rvisualizer.h>
#include <sensor_msgs/JointState.h>
#include <armadillo>

namespace arl
{
namespace viz
{
/**
 * @brief A class which implements a controller for robot visualization in
 * RViz.
 */
class RosStatePublisher : public robot::Controller
{
public:
  /**
   * @brief The default constructor
   *
   * @param r A pointer to the robot for visualization
   * @param rate The rate for the visualization in Hz. Defaults to 50 Hz.
   * @param frame The base frame that which data will be expressed. Defaults to "world".
   * @param topic The topic that RViz reads for the data. Defaults to "/autharl_viz".
   */
  explicit RosStatePublisher(std::shared_ptr<robot::Robot> r, double rate = 50,
                             const std::string& frame = "world", const std::string& topic = "/autharl_viz");

  /**
   * @brief Measures the current state of the robot (Joint Positions).
   */
  void measure();

  /**
   * @brief Publishes the measured joint positions and other data in RViz.
   */
  void command();

  /**
   * @brief Updates ROS data structures with the readings from the robot.
   */
  void update();

  /**
   * @brief The function that runs the controller in the given rate. Should run
   * in a separate thread.
   */
  bool run();

  /**
   * @brief The stop condition of the controller which is that ROS is ok.
   */
  bool stop();

  /**
   * @brief Sets a number of frames for visualization.
   *
   * Another thread can use this function in order to visualize a set of frames
   * in the given loop rate.
   *
   * @param f The frames for visualization
   */
  void setFrame(const std::vector<arma::mat>& f);

private:
  ros::Rate loop_rate;
  ros::Publisher joint_state_pub;
  ros::NodeHandle n;
  sensor_msgs::JointState joint_state;
  KDL::JntArray msr_jnt_pos;

  viz::RVisualizer rviz;

  std::vector<arma::mat> frame;
};
}  // namespace viz
}  // namespace arl

#endif  // AUTHARL_CORE_VIZ_ROS_STATE_PUBLISHER_H
