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

#include <autharl_core/math/plane.h>
#include <kdl/frames_io.hpp>
#include <gtest/gtest.h>

#include <vector>

TEST(CheckCoplanarPoints, CoplanarPointsAreRecognisedAsCoplannar)
{
  std::vector<KDL::Vector> data;
  data.push_back(KDL::Vector(0, 0, 0));
  data.push_back(KDL::Vector(4, 0, 0));
  data.push_back(KDL::Vector(2, 1, 0));
  data.push_back(KDL::Vector(2, 90, 0));
  data.push_back(KDL::Vector(1, 10, 0));
  data.push_back(KDL::Vector(8, 3, 0));

  EXPECT_TRUE(arl::math::areCoplanar(data));

  data.clear();
  data.push_back(KDL::Vector(0, 0, 1));
  data.push_back(KDL::Vector(3, -0.01, 0));
  data.push_back(KDL::Vector(2, 0.02, 999));
  data.push_back(KDL::Vector(2, 0, 21));
  data.push_back(KDL::Vector(1, 0, 12));
  data.push_back(KDL::Vector(8, 0.01, 21));

  EXPECT_TRUE(arl::math::areCoplanar(data, 0.05));
}

TEST(CheckCoplanarPoints, NonCoplanarPointsAreRecognisedAsNonCoplannar)
{
  std::vector<KDL::Vector> data;
  data.push_back(KDL::Vector(0, 0, 8));
  data.push_back(KDL::Vector(4, 0, 10));
  data.push_back(KDL::Vector(2, 1, 100));
  data.push_back(KDL::Vector(2, 90, 12));
  data.push_back(KDL::Vector(1, 10, 21));
  data.push_back(KDL::Vector(8, 3, 321));

  EXPECT_FALSE(arl::math::areCoplanar(data));

  data.clear();
  data.push_back(KDL::Vector(0, 0, 8));
  data.push_back(KDL::Vector(4, 0, 10));
  data.push_back(KDL::Vector(2, 0, 100));
  data.push_back(KDL::Vector(2, 0.1, 12));
  data.push_back(KDL::Vector(1, 0, 21));
  data.push_back(KDL::Vector(8, 0, 321));

  EXPECT_FALSE(arl::math::areCoplanar(data, 0.001));
}

TEST(CheckSortCircularOrder, CheckFourPoints)
{
  std::vector<KDL::Vector> data;
  data.push_back(KDL::Vector(0, 0, 0));
  data.push_back(KDL::Vector(1, 1, 0));
  data.push_back(KDL::Vector(1, 0, 0));
  data.push_back(KDL::Vector(0, 1, 0));

  std::vector<KDL::Vector> output;

  arl::math::sortCircularOrder(data, &output);

  EXPECT_EQ(output.at(0), KDL::Vector(0, 0, 0));
  EXPECT_EQ(output.at(1), KDL::Vector(1, 0, 0));
  EXPECT_EQ(output.at(2), KDL::Vector(1, 1, 0));
  EXPECT_EQ(output.at(3), KDL::Vector(0, 1, 0));
}

TEST(CheckSortCircularOrder, CheckSevenPoints)
{
  std::vector<KDL::Vector> data;
  data.push_back(KDL::Vector(-20, -20, 0));
  data.push_back(KDL::Vector(20, 20, 0));
  data.push_back(KDL::Vector(-15, 10, 0));
  data.push_back(KDL::Vector(5, -15, 0));
  data.push_back(KDL::Vector(-5, 30, 0));
  data.push_back(KDL::Vector(-30, -0, 0));
  data.push_back(KDL::Vector(20, 0, 0));

  std::vector<KDL::Vector> output;

  arl::math::sortCircularOrder(data, &output);

  EXPECT_EQ(output.at(0), KDL::Vector(-20, -20, 0));
  EXPECT_EQ(output.at(1), KDL::Vector(-30, 0, 0));
  EXPECT_EQ(output.at(2), KDL::Vector(-15, 10, 0));
  EXPECT_EQ(output.at(3), KDL::Vector(-5, 30, 0));
  EXPECT_EQ(output.at(4), KDL::Vector(20, 20, 0));
  EXPECT_EQ(output.at(5), KDL::Vector(20, 0, 0));
  EXPECT_EQ(output.at(6), KDL::Vector(5, -15, 0));
}
