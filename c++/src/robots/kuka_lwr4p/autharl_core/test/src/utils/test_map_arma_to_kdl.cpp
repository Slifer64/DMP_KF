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

#include <autharl_core/utils/kdl_arma.h>
#include <gtest/gtest.h>

TEST(TestMapArmaToKdl, TestVector)
{
  arma::vec arma({1, 1, 2});  // NOLINT(whitespace/braces)
  KDL::Vector kdl;
  arl::utils::armaToKdl(arma, &kdl);

  EXPECT_EQ(kdl(0), 1);
  EXPECT_EQ(kdl(1), 1);
  EXPECT_EQ(kdl(2), 2);
}

TEST(TestMapArmaToKdl, TestRotation)
{
  arma::mat arma(3, 3);
  KDL::Rotation kdl;
  arma(0, 0) = 1;
  arma(0, 1) = 2;
  arma(0, 2) = 3;
  arma(1, 0) = 10;
  arma(1, 1) = 20;
  arma(1, 2) = 30;
  arma(2, 0) = 100;
  arma(2, 1) = 200;
  arma(2, 2) = 300;
  arl::utils::armaToKdl(arma, &kdl);

  EXPECT_EQ(kdl(0, 0), 1);
  EXPECT_EQ(kdl(0, 1), 2);
  EXPECT_EQ(kdl(0, 2), 3);
  EXPECT_EQ(kdl(1, 0), 10);
  EXPECT_EQ(kdl(1, 1), 20);
  EXPECT_EQ(kdl(1, 2), 30);
  EXPECT_EQ(kdl(2, 0), 100);
  EXPECT_EQ(kdl(2, 1), 200);
  EXPECT_EQ(kdl(2, 2), 300);
}

TEST(TestMapArmaToKdl, TestFrame)
{
  arma::mat arma(3, 4);
  KDL::Frame kdl;
  arma(0, 0) = 1;
  arma(0, 1) = 2;
  arma(0, 2) = 3;
  arma(0, 3) = 5;
  arma(1, 0) = 10;
  arma(1, 1) = 20;
  arma(1, 2) = 30;
  arma(1, 3) = 6;
  arma(2, 0) = 100;
  arma(2, 1) = 200;
  arma(2, 2) = 300;
  arma(2, 3) = 7;
  arl::utils::armaToKdl(arma, &kdl);

  EXPECT_EQ(kdl.M(0, 0), 1);
  EXPECT_EQ(kdl.M(0, 1), 2);
  EXPECT_EQ(kdl.M(0, 2), 3);
  EXPECT_EQ(kdl.M(1, 0), 10);
  EXPECT_EQ(kdl.M(1, 1), 20);
  EXPECT_EQ(kdl.M(1, 2), 30);
  EXPECT_EQ(kdl.M(2, 0), 100);
  EXPECT_EQ(kdl.M(2, 1), 200);
  EXPECT_EQ(kdl.M(2, 2), 300);
  EXPECT_EQ(kdl.p(0), 5);
  EXPECT_EQ(kdl.p(1), 6);
  EXPECT_EQ(kdl.p(2), 7);
}

TEST(TestMapArmaToKdl, TestWrench)
{
  arma::vec arma(6);
  arma(0) = 1;
  arma(1) = 2;
  arma(2) = 3;
  arma(3) = 0.1;
  arma(4) = 0.2;
  arma(5) = 0.3;
  KDL::Wrench kdl;
  arl::utils::armaToKdl(arma, &kdl);

  EXPECT_EQ(kdl(0), 1);
  EXPECT_EQ(kdl(1), 2);
  EXPECT_EQ(kdl(2), 3);
  EXPECT_EQ(kdl(3), 0.1);
  EXPECT_EQ(kdl(4), 0.2);
  EXPECT_EQ(kdl(5), 0.3);
}

TEST(TestMapArmaToKdl, TestTwist)
{
  arma::vec arma(6);
  arma(0) = 1;
  arma(1) = 2;
  arma(2) = 3;
  arma(3) = 0.1;
  arma(4) = 0.2;
  arma(5) = 0.3;
  KDL::Twist kdl;
  arl::utils::armaToKdl(arma, &kdl);

  EXPECT_EQ(kdl(0), 1);
  EXPECT_EQ(kdl(1), 2);
  EXPECT_EQ(kdl(2), 3);
  EXPECT_EQ(kdl(3), 0.1);
  EXPECT_EQ(kdl(4), 0.2);
  EXPECT_EQ(kdl(5), 0.3);
}

TEST(TestMapArmaToKdl, TestJntArray)
{
  arma::vec arma({0.0, 0.1, 0.2, 0.3, 0.4});  // NOLINT(whitespace/braces)
  KDL::JntArray kdl;
  arl::utils::armaToKdl(arma, &kdl);

  EXPECT_EQ(kdl(0), 0.0);
  EXPECT_EQ(kdl(1), 0.1);
  EXPECT_EQ(kdl(2), 0.2);
  EXPECT_EQ(kdl(3), 0.3);
  EXPECT_EQ(kdl(4), 0.4);
  EXPECT_EQ(kdl.data.size(), 5);
}

TEST(TestMapArmaToKdl, TestJacobian)
{
  arma::mat arma(6, 4);
  arma(0, 0) = 0.0;
  arma(0, 1) = 0.1;
  arma(0, 2) = 0.2;
  arma(0, 3) = 0.3;
  arma(1, 0) = 0.01;
  arma(1, 1) = 0.11;
  arma(1, 2) = 0.21;
  arma(1, 3) = 0.31;
  arma(2, 0) = 0.02;
  arma(2, 1) = 0.12;
  arma(2, 2) = 0.22;
  arma(2, 3) = 0.32;
  arma(3, 0) = 0.03;
  arma(3, 1) = 0.13;
  arma(3, 2) = 0.23;
  arma(3, 3) = 0.33;
  arma(4, 0) = 0.04;
  arma(4, 1) = 0.14;
  arma(4, 2) = 0.24;
  arma(4, 3) = 0.34;
  arma(5, 0) = 0.05;
  arma(5, 1) = 0.15;
  arma(5, 2) = 0.25;
  arma(5, 3) = 0.35;

  KDL::Jacobian kdl;
  arl::utils::armaToKdl(arma, &kdl);
  ASSERT_EQ(kdl.columns(), 4);

  EXPECT_EQ(kdl(0, 0), 0.0);
  EXPECT_EQ(kdl(0, 1), 0.1);
  EXPECT_EQ(kdl(0, 2), 0.2);
  EXPECT_EQ(kdl(0, 3), 0.3);
  EXPECT_EQ(kdl(1, 0), 0.01);
  EXPECT_EQ(kdl(1, 1), 0.11);
  EXPECT_EQ(kdl(1, 2), 0.21);
  EXPECT_EQ(kdl(1, 3), 0.31);
  EXPECT_EQ(kdl(2, 0), 0.02);
  EXPECT_EQ(kdl(2, 1), 0.12);
  EXPECT_EQ(kdl(2, 2), 0.22);
  EXPECT_EQ(kdl(2, 3), 0.32);
  EXPECT_EQ(kdl(3, 0), 0.03);
  EXPECT_EQ(kdl(3, 1), 0.13);
  EXPECT_EQ(kdl(3, 2), 0.23);
  EXPECT_EQ(kdl(3, 3), 0.33);
  EXPECT_EQ(kdl(4, 0), 0.04);
  EXPECT_EQ(kdl(4, 1), 0.14);
  EXPECT_EQ(kdl(4, 2), 0.24);
  EXPECT_EQ(kdl(4, 3), 0.34);
  EXPECT_EQ(kdl(5, 0), 0.05);
  EXPECT_EQ(kdl(5, 1), 0.15);
  EXPECT_EQ(kdl(5, 2), 0.25);
  EXPECT_EQ(kdl(5, 3), 0.35);
}
