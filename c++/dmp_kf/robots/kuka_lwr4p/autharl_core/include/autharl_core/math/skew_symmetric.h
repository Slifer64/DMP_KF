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

#ifndef AUTHARL_CORE_MATH_SKEW_SYMMETRIC_H
#define AUTHARL_CORE_MATH_SKEW_SYMMETRIC_H

#include <kdl/frames.hpp>
#include <Eigen/Core>
#include <armadillo>

namespace arl
{
namespace math
{
/**
 * @brief Calculates the skew symmetrix given a 3D vector using KDL
 *
 * @param v A KDL vector
 * @param skew A KDL rotation matrix as output
 */
inline void skewSymmetric(const KDL::Vector& v, KDL::Rotation* skew)
{
  *skew = KDL::Rotation(0, 0, 0,
                        0, 0, 0,
                        0, 0, 0);
  skew->data[1] = -v(2);
  skew->data[2] =  v(1);
  skew->data[3] =  v(2);
  skew->data[5] = -v(0);
  skew->data[6] = -v(1);
  skew->data[7] =  v(0);
}

/**
 * @brief Calculates the skew symmetrix given a 3D vector using Eigen
 *
 * @param v An Eigen 3D vector
 * @param skew An Eigen 3x3 matrix as output
 */
inline void skewSymmetric(const Eigen::Matrix<double, 3, 1>& v, Eigen::Matrix<double, 3, 3>* skew)
{
  *skew = Eigen::Matrix<double, 3, 3>::Zero();
  (*skew)(0, 1) = -v(2);
  (*skew)(0, 2) =  v(1);
  (*skew)(1, 0) =  v(2);
  (*skew)(1, 2) = -v(0);
  (*skew)(2, 0) = -v(1);
  (*skew)(2, 1) =  v(0);
}

/**
 * @brief Calculates the skew symmetrix given a 3D vector using Armadillo
 *
 * @param v An arma 3D vector
 * @param skew An arma 3x3 matrix as output
 */
inline void skewSymmetric(const arma::vec& v, arma::mat* skew)
{
  *skew = arma::mat(3, 3, arma::fill::zeros);
  (*skew)(0, 1) = -v(2);
  (*skew)(0, 2) =  v(1);
  (*skew)(1, 0) =  v(2);
  (*skew)(1, 2) = -v(0);
  (*skew)(2, 0) = -v(1);
  (*skew)(2, 1) =  v(0);
}
}  // namespace math
}  // namespace arl

#endif  // AUTHARL_CORE_MATH_SKEW_SYMMETRIC_H
