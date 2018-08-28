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

#include <string>
#include <vector>

#include <math.h>
#include <ros/console.h>
#include <limits>

namespace arl
{
namespace robot
{
Model::Model() {}

Model::Model(const std::string& n)
{
  name = n;  // set the name of the robot
}

unsigned int Model::getNrOfJoints(int chain)
{
  if (chain == -1)
  {
    unsigned int output = 0;
    for (unsigned int i = 0; i < num_chains_exclusive; i++)
    {
      output += joint_name.at(i).size();
    }
    return output;
  }
  else if (chain >= 0 && chain <= chain_name.size())
  {
    return joint_name.at(chain).size();
  }
  else
  {
    ROS_ERROR_STREAM("[Model " << this->name << "] "
                     << "Requesting number of joints of a non-existent chain: " << chain);
    return 0;
  }
}


unsigned int Model::getNrOfMainChains()
{
  return num_chains_exclusive;
}

unsigned int Model::getNrOfJoints()
{
  unsigned int output = 0;
  for (unsigned int i = 0; i < num_chains_exclusive; i++)
  {
    output += joint_name.at(i).size();
  }
  return output;
}

std::string Model::getJointName(unsigned int chain, unsigned int joint)
{
  if (chain <= chain_name.size())
  {
    if (joint <= joint_name.at(chain).size())
    {
      return joint_name.at(chain).at(joint);
    }
    else
    {
      ROS_ERROR_STREAM("[Model " << this->name << "] "
                       << "Requesting joint name of a non-existent joint: "
                       << joint << "of chain: " << chain);
      return "";
    }
  }
  else
  {
    ROS_ERROR_STREAM("[Model " << this->name << "] "
                     << "Requesting joint name of a non-existent chain: "
                     << chain);
    return "";
  }
}

std::vector<std::string> Model::getJointNames(unsigned int chain)
{
  if (chain <= this->getNrOfChains())
  {
    return joint_name.at(chain);
  }
  else
  {
    ROS_ERROR_STREAM("[Model " << this->name << "] "
                     << "Requesting joint names of a non-existent chain: "
                     << chain);
    std::vector<std::string> output;
    return output;
  }
}

std::string Model::getJointName(unsigned int joint)
{
  unsigned int temp = 0;
  if (joint < this->getNrOfJoints())
  {
    for (unsigned int i = 0; i < num_chains_exclusive; i++)
    {
      temp += this->getNrOfJoints(i);
      if (joint < temp)
      {
        return joint_name.at(i).at(joint - temp + this->getNrOfJoints(i));
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM("[Model " << this->name << "] "
                     << "Requesting joint name of a non-existent joint: "
                     << joint);
    return "";
  }
}

std::vector<std::string> Model::getJointNames()
{
  std::vector<std::string> output;
  for (unsigned int i = 0; i < num_chains_exclusive; i++)
  {
    for (unsigned int j = 0; j < joint_name.at(i).size(); j++)
    {
      output.push_back(joint_name.at(i).at(j));
    }
  }
  return output;
}

unsigned int Model::getNrOfChains()
{
  return chain_name.size();
}

std::vector<std::string> Model::getChainNames()
{
  return this->chain_name;
}

std::string Model::getChainName(unsigned int chain)
{
  if (chain <= getNrOfChains())
  {
    return chain_name.at(chain);
  }
  else
  {
    ROS_ERROR_STREAM("[Model " << this->name << "] "
                     << "Requesting chain name of a non-existent chain: "
                     << chain);
    return "";
  }
}

std::pair<double, double> Model::getJointLimit(unsigned int chain, unsigned int joint)
{
  if (chain < getNrOfChains() && joint < getNrOfJoints(chain))
  {
    if (chain < this->joint_limit.size() && joint < this->joint_limit.at(chain).size())
    {
      return joint_limit.at(chain).at(joint);
    }
    ROS_WARN_STREAM("[Model " << this->name << "] "
                              << "Requesting non-defined joint limit. Returning default limits: (-pi, pi).");
    std::pair<double, double> output;
    output.first = - M_PI;
    output.second = M_PI;
    return output;
  }
  ROS_ERROR_STREAM("[Model " << this->name << "] "
                             << "Requesting chain/joint limit of a non-existent chain."
                             << " Chain: " << chain
                             << " Joint: " << joint
                             << " Returning NaN limits.");
  std::pair<double, double> output;
  output.first = std::numeric_limits<double>::quiet_NaN();
  output.second = std::numeric_limits<double>::quiet_NaN();
  return output;
}

std::pair<double, double> Model::getJointLimit(unsigned int joint)
{
  unsigned int temp = 0;
  if (joint < this->getNrOfJoints())
  {
    for (unsigned int i = 0; i < num_chains_exclusive; i++)
    {
      temp += this->getNrOfJoints(i);
      if (joint < temp)
      {
        if (joint - temp + this->getNrOfJoints(i) < this->joint_limit.at(i).size())
        {
          return joint_limit.at(i).at(joint - temp + this->getNrOfJoints(i));
        }
        else
        {
          ROS_WARN_STREAM("[Model " << this->name << "] "
                                    << "Requesting non-defined joint limit. Returning default limits: (-pi, pi).");
          std::pair<double, double> output;
          output.first = - M_PI;
          output.second = M_PI;
          return output;
        }
      }
    }
  }
  ROS_ERROR_STREAM("[Model " << this->name << "] "
                             << "Requesting joint limit of a non-existent joint: "
                             << joint);
  std::pair<double, double> output;
  output.first = std::numeric_limits<double>::quiet_NaN();
  output.second = std::numeric_limits<double>::quiet_NaN();
  return output;
}

std::vector<std::pair<double, double>> Model::getJointLimits(unsigned int chain)
{
  if (chain < this->getNrOfChains())
  {
    std::vector<std::pair<double, double>> output;
    for (unsigned int i = 0; i < this->getNrOfJoints(chain); i++)
    {
      output.push_back(getJointLimit(chain, i));
    }
    return output;
  }
  ROS_ERROR_STREAM("[Model " << this->name << "] "
                             << "Requesting joint limits of a non-existent chain: "
                             << chain);
  std::vector<std::pair<double, double>> output;
  return output;
}

std::vector<std::pair<double, double>> Model::getJointLimits()
{
  std::vector<std::pair<double, double>> output;
  for (unsigned int i = 0; i < getNrOfMainChains(); i++)
  {
    for (unsigned int j = 0; j < getNrOfJoints(i); j++)
    {
      output.push_back(this->getJointLimit(i, j));
    }
  }
  return output;
}

unsigned int Model::getGlobalIndex(int chain, unsigned int joint)
{
  if (chain == -1)
  {
    if (joint < getNrOfJoints())
    {
      return joint;
    }
    else
    {
      ROS_ERROR_STREAM("[Model " << this->name << "] "
                                 << "Invalid joint:"
                                 << chain);
    }
  }
  else if (chain > -1 && chain < getNrOfChains())
  {
    if (joint < getNrOfJoints(chain))
    {
      return global_index.at(chain).at(joint);
    }
    else
    {
      ROS_ERROR_STREAM("[Model " << this->name << "] "
                                 << "Invalid joint:"
                                 << joint << " of chain: " << chain);
    }
  }
  else
  {
    ROS_ERROR_STREAM("[Model " << this->name << "] "
                               << "Invalid chain:"
                               << chain);
  }
  return std::numeric_limits<unsigned int>::quiet_NaN();
}
}  // namespace robot
}  // namespace arl
