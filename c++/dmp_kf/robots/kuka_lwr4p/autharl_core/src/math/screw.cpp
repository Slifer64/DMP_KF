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

#include <autharl_core/math/screw.h>
#include <autharl_core/math/skew_symmetric.h>

namespace arl
{
namespace math
{
arma::mat getScrewTransformation(const arma::mat& homogen)
{
  arma::mat screw_transformation(6, 6);
  screw_transformation.submat(0, 0, 2, 2) = homogen.submat(0, 0, 2, 2);
  screw_transformation.submat(3, 3, 5, 5) = homogen.submat(0, 0, 2, 2);
  screw_transformation.submat(0, 3, 2, 5).zeros();

  arma::mat skew;
  skewSymmetric(homogen.submat(0, 3, 2, 3), &skew);
  screw_transformation.submat(3, 0, 5, 2) = skew * homogen.submat(0, 0, 2, 2);
  return screw_transformation;
}

arma::mat get6x6Rotation(const arma::mat& rotation)
{
  arma::mat output(6, 6);
  output.zeros();
  output.submat(0, 0, 2, 2) = rotation;
  output.submat(3, 3, 5, 5) = rotation;
  return output;
}
}  // namespace math
}  // namespace arl
