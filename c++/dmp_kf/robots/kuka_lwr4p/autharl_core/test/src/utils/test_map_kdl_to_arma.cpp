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

TEST(TestMapKdlToArma, TestVector)
{
  KDL::Vector kdl(1, 1, 2);
  arma::vec arma;
  arl::utils::kdlToArma(kdl, &arma);

  EXPECT_EQ(arma(0), 1);
  EXPECT_EQ(arma(1), 1);
  EXPECT_EQ(arma(2), 2);
  EXPECT_EQ(arma.size(), 3);
}

TEST(TestMapKdlToArma, TestRotation)
{
  KDL::Rotation kdl(1, 2, 3, 10, 20, 30, 100, 200, 300);
  arma::mat arma;
  arl::utils::kdlToArma(kdl, &arma);

  EXPECT_EQ(arma(0, 0), 1);
  EXPECT_EQ(arma(0, 1), 2);
  EXPECT_EQ(arma(0, 2), 3);
  EXPECT_EQ(arma(1, 0), 10);
  EXPECT_EQ(arma(1, 1), 20);
  EXPECT_EQ(arma(1, 2), 30);
  EXPECT_EQ(arma(2, 0), 100);
  EXPECT_EQ(arma(2, 1), 200);
  EXPECT_EQ(arma(2, 2), 300);
  EXPECT_EQ(arma.size(), 9);
}

TEST(TestMapKdlToArma, TestFrame)
{
  KDL::Frame kdl(KDL::Rotation(1, 2, 3, 10, 20, 30, 100, 200, 300),
                 KDL::Vector(5, 6, 7));
  arma::mat arma;
  arl::utils::kdlToArma(kdl, &arma);

  EXPECT_EQ(arma(0, 0), 1);
  EXPECT_EQ(arma(0, 1), 2);
  EXPECT_EQ(arma(0, 2), 3);
  EXPECT_EQ(arma(0, 3), 5);
  EXPECT_EQ(arma(1, 0), 10);
  EXPECT_EQ(arma(1, 1), 20);
  EXPECT_EQ(arma(1, 2), 30);
  EXPECT_EQ(arma(1, 3), 6);
  EXPECT_EQ(arma(2, 0), 100);
  EXPECT_EQ(arma(2, 1), 200);
  EXPECT_EQ(arma(2, 2), 300);
  EXPECT_EQ(arma(2, 3), 7);
  EXPECT_EQ(arma.size(), 12);
}

TEST(TestMapKdlToArma, TestWrench)
{
  KDL::Wrench kdl(KDL::Vector(1, 2, 3),
                  KDL::Vector(0.1, 0.2, 0.3));
  arma::vec arma;
  arl::utils::kdlToArma(kdl, &arma);

  EXPECT_EQ(arma(0), 1);
  EXPECT_EQ(arma(1), 2);
  EXPECT_EQ(arma(2), 3);
  EXPECT_EQ(arma(3), 0.1);
  EXPECT_EQ(arma(4), 0.2);
  EXPECT_EQ(arma(5), 0.3);
  EXPECT_EQ(arma.size(), 6);
}

TEST(TestMapKdlToArma, TestTwist)
{
  KDL::Twist kdl(KDL::Vector(1, 2, 3),
                 KDL::Vector(0.1, 0.2, 0.3));
  arma::vec arma;
  arl::utils::kdlToArma(kdl, &arma);

  EXPECT_EQ(arma(0), 1);
  EXPECT_EQ(arma(1), 2);
  EXPECT_EQ(arma(2), 3);
  EXPECT_EQ(arma(3), 0.1);
  EXPECT_EQ(arma(4), 0.2);
  EXPECT_EQ(arma(5), 0.3);
  EXPECT_EQ(arma.size(), 6);
}

TEST(TestMapKdlToArma, TestJntArray)
{
  KDL::JntArray kdl(5);
  kdl(0) = 0.0;
  kdl(1) = 0.1;
  kdl(2) = 0.2;
  kdl(3) = 0.3;
  kdl(4) = 0.4;
  arma::vec arma;
  arl::utils::kdlToArma(kdl, &arma);

  EXPECT_EQ(arma(0), 0.0);
  EXPECT_EQ(arma(1), 0.1);
  EXPECT_EQ(arma(2), 0.2);
  EXPECT_EQ(arma(3), 0.3);
  EXPECT_EQ(arma(4), 0.4);
  EXPECT_EQ(arma.size(), 5);
}

TEST(TestMapKdlToArma, TestJacobian)
{
  KDL::Jacobian kdl(4);
  kdl(0, 0) = 0.0;
  kdl(0, 1) = 0.1;
  kdl(0, 2) = 0.2;
  kdl(0, 3) = 0.3;
  kdl(1, 0) = 0.01;
  kdl(1, 1) = 0.11;
  kdl(1, 2) = 0.21;
  kdl(1, 3) = 0.31;
  kdl(2, 0) = 0.02;
  kdl(2, 1) = 0.12;
  kdl(2, 2) = 0.22;
  kdl(2, 3) = 0.32;
  kdl(3, 0) = 0.03;
  kdl(3, 1) = 0.13;
  kdl(3, 2) = 0.23;
  kdl(3, 3) = 0.33;
  kdl(4, 0) = 0.04;
  kdl(4, 1) = 0.14;
  kdl(4, 2) = 0.24;
  kdl(4, 3) = 0.34;
  kdl(5, 0) = 0.05;
  kdl(5, 1) = 0.15;
  kdl(5, 2) = 0.25;
  kdl(5, 3) = 0.35;

  arma::mat arma;
  arl::utils::kdlToArma(kdl, &arma);
  ASSERT_EQ(arma.size(), 24);

  EXPECT_EQ(arma(0, 0), 0.0);
  EXPECT_EQ(arma(0, 1), 0.1);
  EXPECT_EQ(arma(0, 2), 0.2);
  EXPECT_EQ(arma(0, 3), 0.3);
  EXPECT_EQ(arma(1, 0), 0.01);
  EXPECT_EQ(arma(1, 1), 0.11);
  EXPECT_EQ(arma(1, 2), 0.21);
  EXPECT_EQ(arma(1, 3), 0.31);
  EXPECT_EQ(arma(2, 0), 0.02);
  EXPECT_EQ(arma(2, 1), 0.12);
  EXPECT_EQ(arma(2, 2), 0.22);
  EXPECT_EQ(arma(2, 3), 0.32);
  EXPECT_EQ(arma(3, 0), 0.03);
  EXPECT_EQ(arma(3, 1), 0.13);
  EXPECT_EQ(arma(3, 2), 0.23);
  EXPECT_EQ(arma(3, 3), 0.33);
  EXPECT_EQ(arma(4, 0), 0.04);
  EXPECT_EQ(arma(4, 1), 0.14);
  EXPECT_EQ(arma(4, 2), 0.24);
  EXPECT_EQ(arma(4, 3), 0.34);
  EXPECT_EQ(arma(5, 0), 0.05);
  EXPECT_EQ(arma(5, 1), 0.15);
  EXPECT_EQ(arma(5, 2), 0.25);
  EXPECT_EQ(arma(5, 3), 0.35);
}
