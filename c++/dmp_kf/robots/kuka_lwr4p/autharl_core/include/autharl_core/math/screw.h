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

#ifndef AUTHARL_CORE_MATH_SCREW_H
#define AUTHARL_CORE_MATH_SCREW_H

#include <kdl/frames.hpp>
#include <armadillo>
#include <vector>

namespace arl
{
namespace math
{
/**
* @brief Returns the screw transformation.
*
* The screw transformation (gamma matrix) for transforming a wrench applied on
* frame {B} and expressed in frame {B} to the wrench applied on frame {A} and
* expressed in frame {A}, as given in given by 4.27 in Doulgeri Robotics.  The
* calculation is based on a 3x4 matrix storing the homogeneous transformation
* from tranferring something expressed in frame {B} to something expressed in
* frame {A}. The first three columns should contain the rotation matrix and the
* last column the translation.
*
* @param homogen The homogeneous transformation
* @returns The screw transformation matrix 
*/
arma::mat getScrewTransformation(const arma::mat& homogen);

/**
* @brief Returns the 6x6 matrix which the upper left and the down right
* sumbmatrices are the given 3x3 rotation matrix.
*
* @param rotation The given rotation matrix
* @returns The 6x6 rotation matrix
*/
arma::mat get6x6Rotation(const arma::mat& rotation);

}  // namespace math
}  // namespace arl
#endif  // AUTHARL_CORE_MATH_SCREW_H
