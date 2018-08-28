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

#ifndef AUTHARL_CORE_UTILS_KDL_EIGEN_H
#define AUTHARL_CORE_UTILS_KDL_EIGEN_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Dense>

namespace arl
{
namespace utils
{

inline void kdlToEigen(const KDL::Vector& kdl,
                       Eigen::Vector3d* eigen)
{
  (*eigen)(0) = kdl.x();
  (*eigen)(1) = kdl.y();
  (*eigen)(2) = kdl.z();
}

inline void kdlToEigen(const KDL::Rotation& kdl,
                       Eigen::Matrix3d* eigen)
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      (*eigen)(i, j) = kdl(i, j);
    }
  }
}

inline void kdlToEigen(const KDL::JntArray& kdl,
                       Eigen::VectorXd* eigen)
{
  (*eigen).resize(kdl.rows());
  for (int i = 0; i < kdl.rows(); i++)
  {
    (*eigen)(i) = kdl.data[i];
  }
}

}  // namespace utils
}  // namespace arl
#endif  // AUTHARL_CORE_UTILS_KDL_EIGEN_H
