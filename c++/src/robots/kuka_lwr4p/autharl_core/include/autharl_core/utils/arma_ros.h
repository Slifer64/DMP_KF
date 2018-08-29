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

#ifndef AUTHARL_CORE_UTILS_ARMA_ROS_H
#define AUTHARL_CORE_UTILS_ARMA_ROS_H

#include <armadillo>
#include <sensor_msgs/PointCloud.h>

namespace arl
{
namespace utils
{
/**
 * @brief Converts an arma matrix to a PointCloud ROS msg.
 *
 * The armadillo matrix is assumed to be nx3, for a point cloud with n 3D points.
 *
 * @param kdl The arma matrix as input
 * @param ros The point cloud as ouput
 */
inline void armaToRos(const arma::mat& arma,
                      sensor_msgs::PointCloud* ros,
                      const std::string& frame_id = "world")
{
  unsigned int nr_of_points = arma.n_rows;
  ros->points.resize(nr_of_points);
  for (unsigned int i = 0; i < nr_of_points; i++)
  {
    ros->points.at(i).x = arma(i, 0);
    ros->points.at(i).y = arma(i, 1);
    ros->points.at(i).z = arma(i, 2);
  }
  ros->header.stamp = ros::Time::now();
  ros->header.frame_id = frame_id;
}
}  // namespace utils
}  // namespace arl
#endif  // AUTHARL_CORE_UTILS_ARMA_ROS_H
