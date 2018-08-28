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

#include <autharl_core/math/orientation.h>
#include <gtest/gtest.h>
#include <vector>

TEST(TestOrientationError, OrientationErrorKDLIsCorrect)
{
  std::vector<double> quat1({-0.2989, 0.2176, -0.065, 0.9281});  // NOLINT(whitespace/braces)
  std::vector<double> quat2({1, 0, 0, 0});  // NOLINT(whitespace/braces)

  KDL::Vector output(0.2176, -0.065, 0.9281);
  EXPECT_EQ(arl::math::getOrientationError(quat1, quat2), output);
}

TEST(TestOrientationError, OrientationErrorArmaIsCorrect)
{
  std::vector<double> quat1({-0.2989, 0.2176, -0.065, 0.9281});  // NOLINT(whitespace/braces)
  std::vector<double> quat2({1, 0, 0, 0});  // NOLINT(whitespace/braces)
  arma::vec quat_a(quat1);
  arma::vec quat_b(quat2);

  arma::vec out = arl::math::getOrientationError(quat_a, quat_b);
  EXPECT_EQ(out(0), 0.2176);
  EXPECT_EQ(out(1), -0.065);
  EXPECT_EQ(out(2), 0.9281);
}

TEST(TestQuatToRotTransformation, TransformationIsOk)
{
  arma::vec quat({0.217352, -0.064926, 0.927041, -0.298559});  // NOLINT(whitespace/braces)

  arma::mat out = arl::math::quatToRot(quat);
  EXPECT_NEAR(out(0, 0), -0.8970854008684177, 1e-6);
  EXPECT_NEAR(out(1, 0), -0.2501629893514434, 1e-6);
  EXPECT_NEAR(out(2, 0), -0.3642200740040278, 1e-6);
  EXPECT_NEAR(out(0, 1), 0.00940668062440601, 1e-6);
  EXPECT_NEAR(out(1, 1),  0.8132942350908797, 1e-6);
  EXPECT_NEAR(out(2, 1), -0.5817765907352849, 1e-6);
  EXPECT_NEAR(out(0, 2),  0.4417570575648795, 1e-6);
  EXPECT_NEAR(out(1, 2),  -0.525329388028778, 1e-6);
  EXPECT_NEAR(out(2, 2), -0.7272411815655992, 1e-6);
}

TEST(TestRotToQuatTransformation, TransformationIsOk)
{
  arma::vec quat({0.217352, -0.064926, 0.927041, -0.298559});  // NOLINT(whitespace/braces)
  arma::mat input(3, 3);
  input(0, 0) = -0.8970854008684177;
  input(1, 0) = -0.2501629893514434;
  input(2, 0) = -0.3642200740040278;
  input(0, 1) = 0.00940668062440601;
  input(1, 1) =  0.8132942350908797;
  input(2, 1) = -0.5817765907352849;
  input(0, 2) =  0.4417570575648795;
  input(1, 2) =  -0.525329388028778;
  input(2, 2) = -0.7272411815655992;

  arma::vec out = arl::math::rotToQuat(input);

  EXPECT_NEAR(out(0),  0.21735204890733276, 1e-6);
  EXPECT_NEAR(out(1), -0.06492600712792564, 1e-6);
  EXPECT_NEAR(out(2),   0.9270411017755492, 1e-6);
  EXPECT_NEAR(out(3),  -0.2985590327774135, 1e-6);
}

TEST(TestQuatConjugate, ConjugateIsCorrect)
{
  arma::vec quat({0.217352, -0.064926, 0.927041, -0.298559});  // NOLINT(whitespace/braces)

  arma::vec out = arl::math::getQuatConjugate(quat);

  EXPECT_EQ(out(0), 0.217352);
  EXPECT_EQ(out(1), 0.064926);
  EXPECT_EQ(out(2), -0.927041);
  EXPECT_EQ(out(3), 0.298559);
}
