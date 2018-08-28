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

#include <autharl_core/robot/trajectories.h>

namespace arl
{
namespace robot
{
namespace trajectory
{
arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
{
  arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

  if (t < 0)
  {
    // before start
    retTemp.col(0) = p0;
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp.col(0) = pT;
  }
  else
  {
    // somewhere betweeen ...
    // position
    retTemp.col(0) = p0 +
                     (pT - p0) * (10 * pow(t / totalTime, 3) -
                     15 * pow(t / totalTime, 4) +
                     6 * pow(t / totalTime, 5));
    // vecolity
    retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) -
                     60 * pow(t, 3) / pow(totalTime, 4) +
                     30 * pow(t, 4) / pow(totalTime, 5));
    // acceleration
    retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) -
                     180 * pow(t, 2) / pow(totalTime, 4) +
                     120 * pow(t, 3) / pow(totalTime, 5));
  }

  // return vector
  return retTemp;
}

arma::mat get5thOrderRotationMatrix(double t, arma::mat R0, arma::mat Rt, double totalTime)
{
  arma::mat retTemp = arma::zeros<arma::mat>(R0.n_rows, 5);
  arma::mat R0t = arma::zeros<arma::mat>(3, 3);
  KDL::Rotation R0t_kdl;
  KDL::Vector k;
  arma::mat tmp_thetas, tmp_R;
  arma::vec w_dot, w_ddot;

  if (t < 0)
  {
    // before start
    retTemp(arma::span(0, 2), arma::span(0, 2)) = R0;
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp(arma::span(0, 2), arma::span(0, 2)) = Rt;
  }
  else
  {
    // somewhere betweeen ...
    R0t = R0.t() * Rt;
    arl::utils::armaToKdl(R0t, &R0t_kdl);
    double theta = R0t_kdl.GetRotAngle(k);
    tmp_thetas = get5thOrder(t, arma::vec({0.0}), arma::vec({theta}), totalTime);  // NOLINT(whitespace/braces)
    double curr_theta = tmp_thetas(0, 0);
    double curr_theta_dot = tmp_thetas(0, 1);
    double curr_theta_ddot = tmp_thetas(0, 2);
    // Orientation
    arl::utils::kdlToArma(KDL::Rotation::Rot(k, curr_theta), &tmp_R);
    retTemp.submat(0, 0, 2, 2) = R0 * tmp_R;
    // Angular vecolity
    arl::utils::kdlToArma(k * curr_theta_dot, &w_dot);
    retTemp.col(3) = R0 * w_dot;
    // Angular acceleration
    arl::utils::kdlToArma(k * curr_theta_ddot, &w_ddot);
    retTemp.col(4) = R0 * w_ddot;
  }

  // return matrix
  return retTemp;
}

arma::mat get3rdOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
{
  arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

  if (t < 0)
  {
    // before start
    retTemp.col(0) = p0;
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp.col(0) = pT;
  }
  else
  {
    // somewhere betweeen ...
    // position
    retTemp.col(0) = p0 + (pT - p0) * (-2 * pow(t / totalTime, 3) + 3 * pow(t / totalTime, 2));
    // vecolity
    retTemp.col(1) = (p0 - pT) * (- 6 * pow(t, 2) / pow(totalTime, 3) + 6 * t / pow(totalTime, 2));
    // acceleration
    retTemp.col(2) = (p0 - pT) * (- 12 * t / pow(totalTime, 3) + 12 / pow(totalTime, 2));
  }

  // return vector
  return retTemp;
}
}  // namespace trajectory
}  // namespace robot
}  // namespace arl
