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

#include <autharl_core/robot/arm_hand_controller.h>

#include <vector>
#include <string>
#include <ros/ros.h>

namespace arl
{
namespace robot
{
ArmHandController::ArmHandController() {}

ArmHandController::ArmHandController(std::vector<std::shared_ptr<Robot>> robots) :
  arm(robots.at(0)),
  hand(robots.at(1))
{}

ArmHandController::ArmHandController(const std::string& name,
                                     std::shared_ptr<Robot> arm,
                                     std::shared_ptr<Robot> hand) :
  arm(arm),
  hand(hand)
{
  this->name = name;
}

ArmHandController::ArmHandController(std::shared_ptr<Robot> arm,
                                     std::shared_ptr<Robot> hand) :
  arm(arm),
  hand(hand)
{}

bool ArmHandController::run()
{
  bool arm_is_ok, hand_is_ok;
  arm_is_ok = arm->isOk();
  hand_is_ok = hand->isOk();

  ROS_INFO_STREAM("[AUTh-ARL ArmHandController] " << this->name << "Started.");
  if (arm_is_ok && hand_is_ok)
  {
    ROS_INFO_STREAM("[AUTh-ARL ArmHandController] " << this->name << "Initializing actions.");
    init();
    ROS_INFO_STREAM("[AUTh-ARL ArmHandController] " << this->name << "Starting control loop.");
  }
  while (!stop() && arm_is_ok && hand_is_ok)
  {
    measure();
    update();
    command();
    arm->waitNextCycle();

    arm_is_ok = arm->isOk();
    hand_is_ok = arm->isOk();
  }

  if (!arm_is_ok || !hand_is_ok)
  {
    ROS_ERROR_STREAM("[AUTh-ARL ArmHandController] " << this->name << "Ended due to hardware error.");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("[AUTh-ARL ArmHandController] " << this->name << "Ended.");
  }
}

}  // namespace robot
}  // namespace arl
