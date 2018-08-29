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

#include <autharl_core/math/rotation.h>
#include <vector>
#include <gtest/gtest.h>

class RotationIsLegitTest : public ::testing::Test
{
public:
  std::vector<KDL::Rotation> legit_rot_;
  std::vector<KDL::Rotation> non_unit_rot_;
  std::vector<KDL::Rotation> non_orthogonal_rot_;
  std::vector<KDL::Rotation> non_right_handed_rot_;

  RotationIsLegitTest()
  {
    // Legit rotation matrices
    legit_rot_.push_back(KDL::Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1));
    legit_rot_.push_back(KDL::Rotation(1, 0, 0, 0, 0, 1, 0, -1, 0));
    legit_rot_.push_back(KDL::Rotation(-0.09296994279725346, -0.4400167178340354,  0.8931639702556513,
                                        0.8866143757675725,  -0.4447792483935566, -0.1268320498956797,
                                        0.4530690216937734,   0.7801004475226414,  0.431476248890274));

    // Non unit rotations
    non_unit_rot_.push_back(KDL::Rotation(1.1, 0, 0, 0, 1.4, 0, 0.2, 0, 1));
    non_unit_rot_.push_back(KDL::Rotation(-0.09, -0.44,  0.89,
                                           0.88, -0.44, -0.12,
                                           0.45,  0.78,  0.43));
    non_unit_rot_.push_back(KDL::Rotation(0.32, 0.48, -0.8,
                                       -0.2, 0.60,  0.0,
                                       0.48, 0.64, -0.60));

    // Non orthogonal rotations
    non_orthogonal_rot_.push_back(KDL::Rotation(0.32, 0.32, -0.8,
                                       -0.2, -0.2,  0.0,
                                       0.48, 0.48, -0.60));
    // Non right handed rotations
    non_right_handed_rot_.push_back(KDL::Rotation(-1, 0, 0, 0, 1, 0, 0, 0, 1));
    non_right_handed_rot_.push_back(KDL::Rotation(0.09296994279725346, -0.4400167178340354,  0.8931639702556513,
                                                 -0.8866143757675725,  -0.4447792483935566, -0.1268320498956797,
                                                 -0.4530690216937734,   0.7801004475226414,  0.431476248890274));
  }

  ~RotationIsLegitTest()
  {
  }
};

TEST_F(RotationIsLegitTest, LegitRotationsAreRecognisedAsLegit)
{
  for (int i = 0; i < legit_rot_.size(); i++)
  {
    EXPECT_TRUE(arl::math::rotationHasUnitColumns(legit_rot_[i]))
      << "Fail in " << i;
    EXPECT_TRUE(arl::math::rotationHasOrthogonalColumns(legit_rot_[i]))
      << "Fail in " << i;
    EXPECT_TRUE(arl::math::rotationIsRightHanded(legit_rot_[i]))
      << "Fail in " << i;
    EXPECT_TRUE(arl::math::rotationIsOk(legit_rot_[i]))
      << "Fail in " << i;
  }
}

TEST_F(RotationIsLegitTest, NonUnitRotationsAreRecognisedAsNonLegit)
{
  for (int i = 0; i < non_unit_rot_.size(); i++)
  {
    EXPECT_FALSE(arl::math::rotationHasUnitColumns(non_unit_rot_[i]));
    EXPECT_FALSE(arl::math::rotationIsOk(non_unit_rot_[i]));
  }
}

TEST_F(RotationIsLegitTest, NonOrthogonalRotationsAreRecognisedAsNonLegit)
{
  for (int i = 0; i < non_orthogonal_rot_.size(); i++)
  {
    EXPECT_FALSE(arl::math::rotationHasOrthogonalColumns(non_orthogonal_rot_[i]));
    EXPECT_FALSE(arl::math::rotationIsOk(non_orthogonal_rot_[i]));
  }
}

TEST_F(RotationIsLegitTest, NonRightHandedRotationsAreRecognisedAsNonLegit)
{
  for (int i = 0; i < non_right_handed_rot_.size(); i++)
  {
    EXPECT_FALSE(arl::math::rotationIsRightHanded(non_right_handed_rot_[i]));
    EXPECT_FALSE(arl::math::rotationIsOk(non_right_handed_rot_[i]));
  }
}
