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

#ifndef AUTHARL_CORE_CORE_H
#define AUTHARL_CORE_CORE_H

#include <autharl_core/math.h>
#include <autharl_core/utils.h>
#include <autharl_core/robot.h>
#include <autharl_core/viz.h>

/**
 * @brief The namespace having all the AUTh-ARL codebase
 */
namespace arl
{
/**
 * @brief The namespace having code related to a robot, robot control etc
 *
 * Examples of functionalities are:
 *   - Defined interfaces for robot kinematics, see robot::Model class.
 *   - Defined interfaces for sending or reading to robotic hardware see robot::Robot class.
 *   - Defined interfaces for writing robot controller, see robot::Controller class.
 *   - Generating trajectories, see robot::trajectory namespace
 */
namespace robot
{
/**
 * @brief Contains functions for generating trajectories
 *
 */
namespace trajectory
{
}  // namespace trajectory
}  // namespace robot
/**
 * @brief The namespace having code related to math calculations and tools
 */
namespace math
{
}  // namespace math
/**
 * @brief The namespace having miscellaneous code which does not have a specific category
 *
 * For example transformation between different data types (from KDL to Armadillo).
 */
namespace utils
{
}  // namespace utils
/**
 * @brief The namespace having code relating to visualization tools
 */
namespace viz
{
}  // namespace viz
}  // namespace arl

#endif  // AUTHARL_CORE_CORE_H
