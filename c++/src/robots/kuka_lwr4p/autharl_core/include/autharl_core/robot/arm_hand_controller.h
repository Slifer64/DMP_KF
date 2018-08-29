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

#ifndef AUTHARL_CORE_ROBOT_ARM_HAND_CONTROLLER_H
#define AUTHARL_CORE_ROBOT_ARM_HAND_CONTROLLER_H

#include <autharl_core/robot/robot.h>
#include <autharl_core/robot/controller.h>

#include <string>
#include <vector>

namespace arl
{
namespace robot
{
class ArmHandController : public Controller
{
public:
  ArmHandController();
  explicit ArmHandController(std::vector<std::shared_ptr<Robot>> robots);
  ArmHandController(const std::string& name,
                    std::shared_ptr<Robot> arm,
                    std::shared_ptr<Robot> hand);
  ArmHandController(std::shared_ptr<Robot> arm,
                    std::shared_ptr<Robot> hand);

  bool run();

  std::shared_ptr<Robot> arm;
  std::shared_ptr<Robot> hand;
};
}  // namespace robot
}  // namespace arl

#endif  // AUTHARL_CORE_ROBOT_ARM_HAND_CONTROLLER_H
