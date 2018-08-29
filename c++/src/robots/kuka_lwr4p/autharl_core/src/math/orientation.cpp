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
#include <autharl_core/math/skew_symmetric.h>
#include <vector>

namespace arl
{
namespace math
{
arma::vec getOrientationError(const arma::vec& q, const arma::vec& qd)
{
  arma::vec ep = q.rows(1, 3);
  arma::vec epd = qd.rows(1, 3);

  double n = q(0);
  double nd = qd(0);

  arma::mat skew;
  skewSymmetric(epd, &skew);

  return -n * epd + nd * ep + skew * ep;
}

KDL::Vector getOrientationError(const std::vector<double>& quat,
                                const std::vector<double>& quat_d)
{
  KDL::Vector ep, ep_d;
  ep.data[0] = quat.at(1);
  ep.data[1] = quat.at(2);
  ep.data[2] = quat.at(3);

  ep_d.data[0] = quat_d.at(1);
  ep_d.data[1] = quat_d.at(2);
  ep_d.data[2] = quat_d.at(3);

  double n = quat.at(0);
  double nd = quat_d.at(0);
  KDL::Rotation skew;
  skewSymmetric(ep_d, &skew);
  return -n * ep_d + nd * ep + skew * ep;
}

arma::mat quatToRot(const arma::vec& q)
{
  double n = q(0);
  double ex = q(1);
  double ey = q(2);
  double ez = q(3);

  arma::mat R = arma::eye<arma::mat>(3, 3);

  R(0, 0) = 2 * (n * n + ex * ex) - 1;
  R(0, 1) = 2 * (ex * ey - n * ez);
  R(0, 2) = 2 * (ex * ez + n * ey);

  R(1, 0) = 2 * (ex * ey + n * ez);
  R(1, 1) = 2 * (n * n + ey * ey) - 1;
  R(1, 2) = 2 * (ey * ez - n * ex);

  R(2, 0) = 2 * (ex * ez - n * ey);
  R(2, 1) = 2 * (ey * ez + n * ex);
  R(2, 2) = 2 * (n * n + ez * ez) - 1;

  return R;
}

arma::vec rotToQuat(const arma::mat& R)
{
  arma::vec q = arma::zeros<arma::vec>(4);

  double tr = R(0, 0) + R(1, 1) + R(2, 2);

  if (tr > 0)
  {
    float S = sqrt(tr + 1.0) * 2;  // S=4*qw
    q(0) = 0.25 * S;
    q(1) = (R(2, 1) - R(1, 2)) / S;
    q(2) = (R(0, 2) - R(2, 0)) / S;
    q(3) = (R(1, 0) - R(0, 1)) / S;
  }
  else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2)))
  {
    float S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2;  // S=4*qx
    q(0) = (R(2, 1) - R(1, 2)) / S;
    q(1) = 0.25 * S;
    q(2) = (R(0, 1) + R(1, 0)) / S;
    q(3) = (R(0, 2) + R(2, 0)) / S;
  }
  else if (R(1, 1) > R(2, 2))
  {
    float S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2;  // S=4*qy
    q(0) = (R(0, 2) - R(2, 0)) / S;
    q(1) = (R(0, 1) + R(1, 0)) / S;
    q(2) = 0.25 * S;
    q(3) = (R(1, 2) + R(2, 1)) / S;
  }
  else
  {
    float S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2;  // S=4*qz
    q(0) = (R(1, 0) - R(0, 1)) / S;
    q(1) = (R(0, 2) + R(2, 0)) / S;
    q(2) = (R(1, 2) + R(2, 1)) / S;
    q(3) = 0.25 * S;
  }

  return q / arma::norm(q);
}

arma::vec getQuatConjugate(const arma::vec& q)
{
  arma::vec qout = -q;
  qout(0) = q(0);
  return qout;
}

arma::vec getQuatInverse(const arma::vec& q)
{
  return getQuatConjugate(q) / arma::norm(q);
}

arma::vec getQuatProduct(const arma::vec& q1, const arma::vec& q2)
{
  arma::mat skew;

  arma::vec prod = arma::zeros<arma::vec>(4);

  prod(0) = q1(0) * q2(0) - arma::dot(q1.rows(1, 3) ,  q2.rows(1, 3));
  skewSymmetric(q1.rows(1, 3), &skew);
  prod.rows(1, 3) = q1(0) * q2.rows(1, 3) +  q2(0) * q1.rows(1, 3) + skew * q2.rows(1, 3);

  return prod;
}

arma::vec getQuatDifference(arma::vec q, arma::vec qd)
{
  return getQuatProduct(q, getQuatInverse(qd));
}

}  // namespace math
}  // namespace arl
