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

#ifndef AUTHARL_CORE_MATH_ROTATION_H
#define AUTHARL_CORE_MATH_ROTATION_H

#include <kdl/frames.hpp>

namespace arl
{
namespace math
{
/**
 * @brief Tests if the columns of the rotation matrix are unit vectors
 *
 * @param rot The rotation matrix to be tested
 * @param eps Parameter for the tolerances to rounding errors
 * @returns True if the rotation are unit vectors, false otherwise
 */
bool rotationHasUnitColumns(const KDL::Rotation &rot, double eps = 1e-8);

/**
 * @brief Tests if the columns of the rotation matrix are orthogonal
 *
 * @param rot The rotation matrix to be tested
 * @param eps Parameter for the tolerances to rounding errors
 * @returns True if the rotation are orthogonal, false otherwise
 */
bool rotationHasOrthogonalColumns(const KDL::Rotation &rot, double eps = 1e-8);

/**
 * @brief Tests if the columns of the rotation matrix form a right handed system
 *
 * @param rot The rotation matrix to be tested
 * @param eps Parameter for the tolerances to rounding errors
 * @returns True if the rotation is right-handed, false otherwise
 */
bool rotationIsRightHanded(const KDL::Rotation &rot, double eps = 1e-8);

/**
 * @brief Tests if a 3x3 matrix R is a legit rotation matrix, i.e. if R belongs to SO(3).
 *
 * This function tests if the columns of the rotation matrix:
 *   - Are unit vectors
 *   - Are orthogonal to each other
 *   - Forms a righthanded system
 *
 * @param rot The rotation matrix to be tested
 * @param eps Parameter for the tolerances to rounding errors
 * @returns True if the rotation is ok, false otherwise
 */
bool rotationIsOk(const KDL::Rotation &rot, double eps = 1e-8);
}  // namespace math
}  // namespace arl

#endif  // AUTHARL_CORE_MATH_ROTATION_H
