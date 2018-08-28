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

#ifndef AUTHARL_CORE_ROBOT_TRAJECTORIES_H
#define AUTHARL_CORE_ROBOT_TRAJECTORIES_H

#include <armadillo>
#include <autharl_core/utils/kdl_arma.h>

namespace arl
{
namespace robot
{
namespace trajectory
{
/**
 * @brief Designs a 5th order trajectory
 *
 * @param t For the current time instance
 * @param p0 From this point (initial)
 * @param pT To the target
 * @param totalTime If the total duaration of motion is this
 * @return The exact position, velocity and acceleration for that time
 * instance, as columns [p][pdot][pddot]
 */
arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime);

/**
 * @brief Designs a 5th order trajectory for orientation
 *
 * @param t For the current time instance
 * @param R0 Initial orientation
 * @param Rt Target orientation
 * @param totalTime If the total duration of motion is this
 * @return The exact orientation[3x3], angular velocity[3x1] and angular acceleration[3x1]
 * for that time instance, as columns [R][qdot][qddot]
 */
arma::mat get5thOrderRotationMatrix(double t, arma::mat R0, arma::mat Rt, double totalTime);

/**
 * @brief Designs a 3th order trajectory
 * @param t For the current time instance
 * @param p0 From this point (initial)
 * @param pT To the target
 * @param totalTime If the total duaration of motion is this
 * @return The exact position, velocity and acceleration for that time
 * instance, as columns [p][pdot][pddot]
 */
arma::mat get3rdOrder(double t, arma::vec p0, arma::vec pT, double totalTime);

}  // namespace trajectory
}  // namespace robot
}  // namespace arl
#endif  // AUTHARL_CORE_ROBOT_TRAJECTORIES_H
