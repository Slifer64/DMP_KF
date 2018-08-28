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

#ifndef AUTHARL_CORE_VIZ_RVISUALIZER_H
#define AUTHARL_CORE_VIZ_RVISUALIZER_H

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <kdl/frames.hpp>
#include <armadillo>

#include <string>
#include <vector>

using rviz_visual_tools::colors;
using rviz_visual_tools::scales;
namespace arl
{
namespace viz
{
static const std::string DEFAULT_RVIZ_TOPIC = "/autharl_viz";
static const std::string DEFAULT_BASE_FRAME = "map";

/**
 * @brief This class implements visualization functionalities for RViz.
 */
class RVisualizer : public rviz_visual_tools::RvizVisualTools
{
public:
  /**
   * @brief The default constructor which initializes the object with the
   * default topic and base frame
   */
  RVisualizer();

  /**
   * @brief The constructor used in order to specify different base frame or
   * topic than the defaults.
   *
   * @param base_frame The name of the base frame
   * @param rviz_topic The name of the ROS topic
   */
  RVisualizer(const std::string& base_frame,
              const std::string& rviz_topic = DEFAULT_RVIZ_TOPIC);

  /**
   * @brief Visualizes a set of points as a connected path in RViz. The points
   * can be any type which implements the operator().
   *
   * @param points The set of points to be visualized
   * @param color The color of the path defined in the enum
   *        rviz_visual_tools::colors.
   * @param radius The width of path
   * @param alpha The opacity of the path from 0 to 1
   * @param lifetime The amount of time to be visualized in seconds. 0 stands
   *        for infinite time
   */
  template<typename T> void visualizePath(const std::vector<T>& points,
                                          rviz_visual_tools::colors color = rviz_visual_tools::colors::RED,
                                          double radius = 0.01,
                                          double alpha = 1,
                                          double lifetime = 0)
  {
    std::vector<Eigen::Vector3d> p(points.size());
    std::vector<geometry_msgs::Pose> path;
    for (int i = 0; i < points.size(); i++)
    {
      for (int j = 0; j < 3; j++) p.at(i)(j) = points.at(i)(j);
      path[i].position.x = p.at(i)(0);
      path[i].position.y = p.at(i)(1);
      path[i].position.z = p.at(i)(2);
      path[i].orientation.w = 1.0;
      path[i].orientation.x = 0.0;
      path[i].orientation.y = 0.0;
      path[i].orientation.z = 0.0;
    }

    setAlpha(alpha);
    setLifetime(lifetime);
    //this->publishPath(path, color, rviz_visual_tools::MEDIUM);
    this->triggerBatchPublish();
  }

  /**
   * @brief Visualizes a a sphere in RViz. The center of the sphere can be any
   * type which implements the operator().
   *
   * @param center The center of the sphere in m
   * @param radius The radius of the sphere in m
   * @param color The color of the path defined in the enum
   *        rviz_visual_tools::colors.
   * @param alpha The opacity of the path from 0 to 1
   * @param lifetime The amount of time to be visualized in seconds. 0 stands
   *        for infinite time
   */
  template<typename T> void visualizeSphere(const T& center,
                                            const double radius,
                                            rviz_visual_tools::colors color = rviz_visual_tools::colors::RED,
                                            double alpha = 1,
                                            double lifetime = 0)
  {
    Eigen::Vector3d p;
    for (int j = 0; j < 3; j++)
    {
      p(j) =  center(j);
    }

    setAlpha(alpha);
    setLifetime(lifetime);
    this->publishSphere(p, color, 2.0 * radius);
    this->triggerBatchPublish();
  }

  /**
   * @brief Visualizes a set of spheres in RViz. The centers of the sphere can
   * be any type which implements the operator().
   *
   * @param centers A vector with the centers of the sphere in m
   * @param radii The radii of the spheres in m
   * @param color The color of the path defined in the enum
   *        rviz_visual_tools::colors.
   * @param alpha The opacity of the path from 0 to 1
   * @param lifetime The amount of time to be visualized in seconds. 0 stands
   *        for infinite time
   */
  template<typename T> void visualizeSpheres(const std::vector<T>& centers,
                                             const std::vector<double>& radii,
                                             rviz_visual_tools::colors color = rviz_visual_tools::colors::RED,
                                             double alpha = 1,
                                             double lifetime = 0)
  {
    // Error handling
    if (centers.size() != radii.size())
    {
      std::cout << "[arl::viz::RVizualizer] "
                << "You need to provide a vector of centers and with equal size with the vectors of radii" << std::endl;
      return;
    }

    std::vector<Eigen::Vector3d> p(centers.size());
    for (int i = 0; i < centers.size(); i++)
    {
      for (int j = 0; j < 3; j++)
      {
        p.at(i)(j) =  centers.at(i)(j);
      }
    }

    setAlpha(alpha);
    setLifetime(lifetime);
    for (int i = 0; i < centers.size(); i++)
    {
      this->publishSphere(p.at(i), color, 2.0 * radii.at(i));
    }
    this->triggerBatchPublish();
  }

  /**
   * @brief Visualizes an armadillo frame given as a 3x4 homogeneous
   * transformation, having the rotation matrix in the first 3 columns and the
   * position of the origin on the last column
   *
   * @param pose The homogeneous transformation
   * @param length The length of the axes
   * @param width The width of the axes
   * @param alpha The opacity from 0 to 1
   * @param lifetime The amount of time to be visualized in seconds. 0 stands
   *        for infinite time
   */
  void visualizeFrame(const arma::mat& pose, double length = 0.1,
                      double width = 0.01, double alpha = 1,
                      double lifetime = 0);

  /**
   * @brief Visualizes a KDL frame.
   *
   * @param pose The pose of the frame
   * @param length The length of the axes
   * @param width The width of the axes
   * @param alpha The opacity from 0 to 1
   * @param lifetime The amount of time to be visualized in seconds. 0 stands
   *        for infinite time
   */
  void visualizeFrame(const KDL::Frame& pose, double length = 0.1,
                      double radius = 0.01, double alpha = 1,
                      double lifetime = 0);

  void visualizeText(const arma::mat& pose,
                     const std::string& text,
                     rviz_visual_tools::colors color = rviz_visual_tools::colors::WHITE,
                     rviz_visual_tools::scales scale = rviz_visual_tools::scales::REGULAR,
                     double alpha = 1,
                     double lifetime = 0);
  /**
   * @brief Visualizes a cylinder in RViz. Needs a line segment in space (two
   * points) which implement the operator() and its radius.
   *
   * @param point_1 The first point of the line segment
   * @param point_2 The second point of the line segment
   * @param radius The radius of the cylinder in m
   * @param color The color of the path defined in the enum
   *        rviz_visual_tools::colors.
   * @param alpha The opacity of the path from 0 to 1
   * @param lifetime The amount of time to be visualized in seconds. 0 stands
   *        for infinite time
   */
  template<typename T> void visualizeCylinder(const T& point_1,
                                              const T& point_2,
                                              const double radius,
                                              rviz_visual_tools::colors color = rviz_visual_tools::colors::RED,
                                              double alpha = 1,
                                              double lifetime = 0)
  {
    Eigen::Vector3d p1, p2;
    for (int j = 0; j < 3; j++)
    {
      p1(j) =  point_1(j);
      p2(j) =  point_2(j);
    }

    setAlpha(alpha);
    setLifetime(lifetime);
    this->publishCylinder(p1, p2, color, radius);
    this->triggerBatchPublish();
  }

  /**
   * @brief Visualizes a vector/arrow in RViz. Needs two 3D points in space
   * which represent the start and the the end of the vector.
   *
   * @param start The start of the vector
   * @param end The end of the vector
   * @param width The width of the vector
   * @param color The color of the path defined in the enum
   *        rviz_visual_tools::colors.
   * @param alpha The opacity of the path from 0 to 1
   * @param lifetime The amount of time to be visualized in seconds. 0 stands
   *        for infinite time
   */
  template<typename T> void visualizeVector(const T& start,
                                            const T& end,
                                            double width = 0.05,
                                            rviz_visual_tools::colors color = rviz_visual_tools::colors::RED,
                                            double alpha = 1,
                                            double lifetime = 0)
  {
    vector_marker_.header.stamp = ros::Time::now();

    vector_marker_.id++;

    vector_marker_.scale.x = width;
    vector_marker_.scale.y = width*2;
    vector_marker_.scale.z = width*3;
    vector_marker_.color = getColor(color);
    vector_marker_.color.a = alpha;  // Don't forget to set the alpha!
    vector_marker_.lifetime = ros::Duration(lifetime);

    vector_marker_.points.clear();
    geometry_msgs::Point temp_point;
    temp_point.x = start(0);
    temp_point.y = start(1);
    temp_point.z = start(2);
    vector_marker_.points.push_back(temp_point);
    temp_point.x = end(0);
    temp_point.y = end(1);
    temp_point.z = end(2);
    vector_marker_.points.push_back(temp_point);

    publishMarker(vector_marker_);
    this->triggerBatchPublish();
  }

  /**
   * @brief Visualizes a cuboid (a paralliped box) in RViz. Needs two 3D points in space
   * which represent the top corner of the box and the bottom opposite corner.
   *
   * @param top The top corner of the box
   * @param bottom The bottom corner of the box
   * @param color The color of the path defined in the enum
   *        rviz_visual_tools::colors.
   * @param alpha The opacity of the path from 0 to 1
   * @param lifetime The amount of time to be visualized in seconds. 0 stands
   *        for infinite time
   */
  template<typename T> void visualizeBox(const T& top,
                                         const T& bottom,
                                         rviz_visual_tools::colors color = rviz_visual_tools::colors::RED,
                                         double alpha = 1,
                                         double lifetime = 0)
  {
    Eigen::Vector3d p1, p2;
    for (int j = 0; j < 3; j++)
    {
      p1(j) =  top(j);
      p2(j) =  bottom(j);
    }

    setAlpha(alpha);
    setLifetime(lifetime);
    this->publishCuboid(p1, p2, color);
    this->triggerBatchPublish();
  }

  visualization_msgs::Marker vector_marker_;
};
}  // namespace viz
}  // namespace arl

#endif  // AUTHARL_CORE_VIZ_RVISUALIZER_H
