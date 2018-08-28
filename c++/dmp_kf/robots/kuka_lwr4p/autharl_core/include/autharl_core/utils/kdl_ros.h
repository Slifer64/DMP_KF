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

#ifndef AUTHARL_CORE_UTILS_KDL_ROS_H
#define AUTHARL_CORE_UTILS_KDL_ROS_H

#include <kdl/frames.hpp>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

namespace arl
{
namespace utils
{
/**
 * @brief Converts a ROS vector (geometry_msgs Vector3) to a KDL Vector
 *
 * @param ros The ROS vector as input
 * @param kdl The KDL vector as ouput
 * @todo To be implemented
 */
inline void rosToKdl(const geometry_msgs::Vector3& ros,
                    KDL::Vector* kdl)
{
}

/**
 * @brief Converts a ROS Point (geometry_msgs Point) to a KDL Vector
 *
 * @param ros The ROS point as input
 * @param kdl The KDL vector as ouput
 * @todo To be implemented
 */
inline void rosToKdl(const geometry_msgs::Point& ros,
                    KDL::Vector* kdl)
{
}

/**
 * @brief Converts a ROS Quaternion (geometry_msgs Quaternion) to a KDL
 * Rotation. It will perform the conversion even if the quaternion is not unit
 *
 * @param ros The ROS point as input
 * @param kdl The KDL vector as ouput
 * @param eps The tolerance in errors
 * @return True if the quaternion is unit, otherwise false
 * @todo To be implemented
 */
inline bool rosToKdl(const geometry_msgs::Quaternion& ros,
                    KDL::Rotation* kdl,
                    double eps = 1e-6)
{
}

/**
 * @brief Converts a ROS Pose (geometry_msgs Pose) to a KDL Frame. It will
 * perform the conversion even if the quaternion of the pose is not unit
 *
 * @param ros The ROS Pose as input
 * @param kdl The KDL Frame as ouput
 * @param eps The tolerance in errors
 * @return True if the quaternion of the pose is unit, otherwise false
 * @todo To be implemented
 */
inline bool rosToKdl(const geometry_msgs::Pose& ros,
                    KDL::Frame* kdl,
                    double eps = 1e-6)
{
}
/**
 * @brief Converts a KDL vector to a ROS Vector (geometry_msgs Vector3)
 *
 * @param kdl The KDL vector as input
 * @param ros The ROS vector as output
 * @todo To be implemented
 */
inline void kdlToRos(const KDL::Vector& kdl,
                    geometry_msgs::Vector3* ros)
{
}

/**
 * @brief Converts a KDL vector to a ROS Point (geometry_msgs Point)
 *
 * @param kdl The KDL vector as input
 * @param ros The ROS point as output
 * @todo To be implemented
 */
inline void kdlToRos(const KDL::Vector& kdl,
                    geometry_msgs::Point* ros)
{
}

/**
 * @brief Converts a KDL Rotation to a ROS Quaternion (geometry_msgs
 * Quaternion). It will perform the conversion even if the rotation is not
 * legit.
 *
 * @param kdl The KDL Rotation as input
 * @param ros The ROS Quaternion as output
 * @param eps The tolerance in errors
 * @return True if the rotation is legit, false if the rotation is not legit
 * @todo To be implemented
 */
inline bool kdlToRos(const KDL::Rotation& kdl,
                    geometry_msgs::Quaternion* ros,
                    double eps = 1e-6)
{
}

/**
 * @brief Converts a KDL Frame to a ROS pose (geometry_msgs Pose). It will
 * perform the conversion even if the rotation matrix of the frame is not
 * legit.
 *
 * @param kdl The KDL Frame as input
 * @param ros The ROS Pose as output
 * @param eps The tolerance in errors
 * @return True if the rotation of the frame is legit, false if the rotation is not legit
 * @todo To be implemented
 */
inline bool kdlToRos(const KDL::Frame& kdl,
                    geometry_msgs::Pose* ros,
                    double eps = 1e-6)
{
}
}  // namespace utils
}  // namespace arl
#endif  // AUTHARL_CORE_UTILS_KDL_ROS_H
