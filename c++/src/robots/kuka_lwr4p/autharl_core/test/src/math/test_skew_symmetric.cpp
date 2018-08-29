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

#include <autharl_core/math/skew_symmetric.h>
#include <kdl/frames_io.hpp>
#include <gtest/gtest.h>

TEST(SkewSymmetric, TestKDLVersion)
{
  KDL::Vector input(1, 2, 3);
  KDL::Rotation output;
  KDL::Rotation result(0, -3, 2, 3, 0, -1, -2, 1, 0);
  arl::math::skewSymmetric(input, &output);
  EXPECT_EQ(output, result);
}

TEST(SkewSymmetric, TestEigenVersion)
{
  Eigen::Matrix<double, 3, 1> input(1, 2, 3);
  Eigen::Matrix<double, 3, 3> output;
  Eigen::Matrix<double, 3, 3> result;
  result << 0, -3, 2, 3, 0, -1, -2, 1, 0;
  arl::math::skewSymmetric(input, &output);
  EXPECT_EQ(output, result);
}

TEST(SkewSymmetric, TestArmaVersion)
{
  arma::vec input = {1, 2, 3};
  arma::mat output;
  arma::mat result(3, 3);
  result << 0 << -3 << 2 << arma::endr
         << 3 << 0 << -1 << arma::endr
         << -2 << 1 << 0 << arma::endr;
  arl::math::skewSymmetric(input, &output);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      EXPECT_EQ(result(i, j), output(i, j));
    }
  }
}
