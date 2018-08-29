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

#include <autharl_core/robot/controller.h>

#include <vector>
#include <string>

namespace arl
{
namespace robot
{
Controller::Controller() {}

Controller::Controller(std::shared_ptr<Robot> r,
                       const std::string& name) :
  robot(r),
  name(name)
{}

std::string Controller::getName()
{
  return name;
}


void Controller::measure()
{
}

void Controller::command()
{
}

bool Controller::run()
{
  // a mutex in robot should be locked to ensure no other controller is running
  // on this robot
  init();
  while (!stop() &&
         robot->isOk() &&
         !this->external_stop)
  {
    measure();
    update();
    command();
    robot->waitNextCycle();
  }
  return success();
}

void Controller::init() {}

bool Controller::success()
{
  return true;
}

bool Controller::stop()
{
  return false;
}

void Controller::setExternalStop(bool arg)
{
  this->external_stop = arg;
}

}  // namespace robot
}  // namespace arl
