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

#include <autharl_core/robot/model.h>
#include <gtest/gtest.h>
#include <vector>
#include <cmath>

class TestModel: public arl::robot::Model
{
public:
  TestModel()
  {
    name = "Test Robot Model";
    num_chains_exclusive = 4;
    chain_name.push_back("arm");
    chain_name.push_back("finger_1");
    chain_name.push_back("finger_2");
    chain_name.push_back("finger_3");
    chain_name.push_back("arm_to_finger_1");
    chain_name.push_back("arm_to_finger_2");

    joint_name.push_back({"arm_1", "arm_2", "arm_3", "arm_4"});  // NOLINT(whitespace/braces)
    joint_name.push_back({"proximal_1", "distal_1"});  // NOLINT(whitespace/braces)
    joint_name.push_back({"proximal_2", "distal_2"});  // NOLINT(whitespace/braces)
    joint_name.push_back({"proximal_3", "distal_3"});  // NOLINT(whitespace/braces)
    joint_name.push_back({"arm_1", "arm_2", "arm_3", "arm_4", "proximal_1", "distal_1"});  // NOLINT(whitespace/braces)
    joint_name.push_back({"arm_1", "arm_2", "arm_3", "arm_4", "proximal_2", "distal_2"});  // NOLINT(whitespace/braces)

    joint_limit.push_back({std::make_pair(-0.1, 0.1),  // NOLINT(whitespace/braces)
                           std::make_pair(-0.2, 0.2),
                           std::make_pair(-0.3, 0.3),
                           std::make_pair(-0.4, 0.4)});  // NOLINT(whitespace/braces)
    joint_limit.push_back({std::make_pair(-0.01, 0.01),  // NOLINT(whitespace/braces)
                           std::make_pair(-0.02, 0.02)});  // NOLINT(whitespace/braces)
    joint_limit.push_back({std::make_pair(-0.5, 0.5),  // NOLINT(whitespace/braces)
                           std::make_pair(-0.6, 0.6)});  // NOLINT(whitespace/braces)
    joint_limit.push_back({std::make_pair(-1, 1)});  // NOLINT(whitespace/braces)
  }
};

TEST(TestModel, TestGettersOfChainsAndJoints)
{
  TestModel test_model;

  EXPECT_EQ(test_model.getNrOfChains(), 6);
  EXPECT_EQ(test_model.getNrOfMainChains(), 4);

  std::vector<std::string> output;
  output = test_model.getChainNames();
  EXPECT_EQ(output.at(0), "arm");
  EXPECT_EQ(output.at(1), "finger_1");
  EXPECT_EQ(output.at(2), "finger_2");
  EXPECT_EQ(output.at(3), "finger_3");
  EXPECT_EQ(output.at(4), "arm_to_finger_1");
  EXPECT_EQ(output.at(5), "arm_to_finger_2");

  EXPECT_EQ(test_model.getChainName(0), "arm");
  EXPECT_EQ(test_model.getChainName(1), "finger_1");
  EXPECT_EQ(test_model.getChainName(2), "finger_2");
  EXPECT_EQ(test_model.getChainName(3), "finger_3");
  EXPECT_EQ(test_model.getChainName(4), "arm_to_finger_1");
  EXPECT_EQ(test_model.getChainName(5), "arm_to_finger_2");

  EXPECT_EQ(test_model.getNrOfJoints(), 10);
  EXPECT_EQ(test_model.getNrOfJoints(0), 4);
  EXPECT_EQ(test_model.getNrOfJoints(1), 2);
  EXPECT_EQ(test_model.getNrOfJoints(2), 2);
  EXPECT_EQ(test_model.getNrOfJoints(3), 2);
  EXPECT_EQ(test_model.getNrOfJoints(4), 6);
  EXPECT_EQ(test_model.getNrOfJoints(5), 6);

  output = test_model.getJointNames();
  EXPECT_EQ(output.at(0), "arm_1");
  EXPECT_EQ(output.at(1), "arm_2");
  EXPECT_EQ(output.at(2), "arm_3");
  EXPECT_EQ(output.at(3), "arm_4");
  EXPECT_EQ(output.at(4), "proximal_1");
  EXPECT_EQ(output.at(5), "distal_1");
  EXPECT_EQ(output.at(6), "proximal_2");
  EXPECT_EQ(output.at(7), "distal_2");
  EXPECT_EQ(output.at(8), "proximal_3");
  EXPECT_EQ(output.at(9), "distal_3");

  EXPECT_EQ(test_model.getJointName(0), "arm_1");
  EXPECT_EQ(test_model.getJointName(1), "arm_2");
  EXPECT_EQ(test_model.getJointName(2), "arm_3");
  EXPECT_EQ(test_model.getJointName(3), "arm_4");
  EXPECT_EQ(test_model.getJointName(4), "proximal_1");
  EXPECT_EQ(test_model.getJointName(5), "distal_1");
  EXPECT_EQ(test_model.getJointName(6), "proximal_2");
  EXPECT_EQ(test_model.getJointName(7), "distal_2");
  EXPECT_EQ(test_model.getJointName(8), "proximal_3");
  EXPECT_EQ(test_model.getJointName(9), "distal_3");
  EXPECT_EQ(test_model.getJointName(10), "");

  output = test_model.getJointNames(0);
  EXPECT_EQ(output.at(0), "arm_1");
  EXPECT_EQ(output.at(1), "arm_2");
  EXPECT_EQ(output.at(2), "arm_3");
  EXPECT_EQ(output.at(3), "arm_4");
  output = test_model.getJointNames(2);
  EXPECT_EQ(output.at(0), "proximal_2");
  EXPECT_EQ(output.at(1), "distal_2");

  EXPECT_EQ(test_model.getJointName(0, 0), "arm_1");
  EXPECT_EQ(test_model.getJointName(0, 1), "arm_2");
  EXPECT_EQ(test_model.getJointName(0, 2), "arm_3");
  EXPECT_EQ(test_model.getJointName(0, 3), "arm_4");
  EXPECT_EQ(test_model.getJointName(1, 0), "proximal_1");
  EXPECT_EQ(test_model.getJointName(1, 1), "distal_1");
  EXPECT_EQ(test_model.getJointName(2, 0), "proximal_2");
  EXPECT_EQ(test_model.getJointName(2, 1), "distal_2");
  EXPECT_EQ(test_model.getJointName(3, 0), "proximal_3");
  EXPECT_EQ(test_model.getJointName(3, 1), "distal_3");
}

TEST(TestModel, TestJointLimits)
{
  TestModel test_model;

  EXPECT_EQ(test_model.getJointLimit(0, 1).first, -0.2);
  EXPECT_EQ(test_model.getJointLimit(1, 1).second, 0.02);
  EXPECT_EQ(test_model.getJointLimit(3, 1).first, -M_PI);
  EXPECT_EQ(test_model.getJointLimit(4, 1).first, -M_PI);
  EXPECT_TRUE(std::isnan(test_model.getJointLimit(4, 10).first));


  EXPECT_EQ(test_model.getJointLimit(0).first, -0.1);
  EXPECT_EQ(test_model.getJointLimit(1).second, 0.2);
  EXPECT_EQ(test_model.getJointLimit(3).first, -0.4);
  EXPECT_EQ(test_model.getJointLimit(4).first, -0.01);
  EXPECT_EQ(test_model.getJointLimit(9).first, -M_PI);
  EXPECT_TRUE(std::isnan(test_model.getJointLimit(100).first));

  std::vector<std::pair<double, double>> output;
  output = test_model.getJointLimits(0);
  EXPECT_EQ(output.at(0).first, -0.1);
  EXPECT_EQ(output.at(1).first, -0.2);
  EXPECT_EQ(output.at(2).first, -0.3);
  EXPECT_EQ(output.at(3).first, -0.4);

  output = test_model.getJointLimits(40);
  EXPECT_TRUE(output.empty());

  output = test_model.getJointLimits(3);
  EXPECT_EQ(output.at(0).first, -1);
  EXPECT_EQ(output.at(0).second, 1);
  EXPECT_EQ(output.at(1).first, -M_PI);
  EXPECT_EQ(output.at(1).second, M_PI);

  output = test_model.getJointLimits();
  EXPECT_EQ(output.at(0).first, -0.1);
  EXPECT_EQ(output.at(1).first, -0.2);
  EXPECT_EQ(output.at(9).first, -M_PI);
  EXPECT_EQ(output.at(9).second, M_PI);
}

