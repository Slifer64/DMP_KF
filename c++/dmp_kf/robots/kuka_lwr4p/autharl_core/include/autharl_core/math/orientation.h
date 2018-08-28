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

#ifndef AUTHARL_CORE_MATH_ORIENTATION_H
#define AUTHARL_CORE_MATH_ORIENTATION_H

#include <kdl/frames.hpp>
#include <armadillo>
#include <vector>

namespace arl
{
namespace math
{
/**
* @brief Returns the orientation error between two quaternions using Armadillo.
*
* The quaternions are following the convention w, x, y, z. The returned
* quaternion error is given by Eq. 3.91 at p. 140 of book "Robotics: Modelling,
* Planning and Control of Siciliano", with different sequence.
*
* @param q The current orientation
* @param qd The desired orientation
* @returns The quaternion error
*
* @see getOrientationError(const std::vector<double>& quat, const
* std::vector<double>& quat_d);
*/
arma::vec getOrientationError(const arma::vec& q, const arma::vec& qd);

/**
 * @brief Calculate the orientation error between two quaternions using STL and KDL.
 *
 * The quaternions are following the convention w, x, y, z. The returned
 * quaternion error is given by Eq. 3.91 at p. 140 of book "Robotics: Modelling,
 * Planning and Control of Siciliano", with different sequence.
 *
 * @param quat The current orientation
 * @param quat_d The desired orientation
 * @return The quaternion error as a KDL Vector
 *
 * @see getOrientationError(const arma::vec& q, const arma::vec& qd)
 */
KDL::Vector getOrientationError(const std::vector<double>& quat,
                                const std::vector<double>& quat_d);

/**
 * @brief Transforms a quaternion to a rotation matrix.
 *
 * The quaternion should be given following the convention: w, x, y, z
 *
 * @param q The quaternion to be transformed
 * @returns The resulting rotation matrix
 */
arma::mat quatToRot(const arma::vec& q);

/**
 * @brief Transforms a rotation matrix to a quaternion.
 *
 * The quaternion will returned following the convention: w, x, y, z
 *
 * @param R A 3x3 armadillo matrix as the rotation matrix
 * @return The quaternion as output
 */
arma::vec rotToQuat(const arma::mat& R);


/**
 * @brief Calculates the quaternion conjugate.
 *
 * The quaternions as input and output follows the convention w, x, y, z
 *
 * @param q The given quaternion.
 * @return The quaternion conjugate
 */
arma::vec getQuatConjugate(const arma::vec& q);

/**
 * @brief Calculates the inverse of a given quaternion.
 *
 * The quaternions as input and output follows the convention w, x, y, z
 *
 * @param q The given quaternion
 * @return The inverse of the quaternion
 */
arma::vec getQuatInverse(const arma::vec& q);


/**
 * @brief Calculates the product of two quaternions.
 *
 * The quaternions as input and output follows the convention w, x, y, z
 *
 * @param q1 Quaternion input 1
 * @param q1 Quaternion input 2
 * @return The quaternion product
 */
arma::vec getQuatProduct(const arma::vec& q1, const arma::vec& q2);

/**
 * @brief Calculates the difference of two quaternions.
 *
 * The quaternions as input and output follows the convention w, x, y, z
 *
 * @param q Quaternion input 1
 * @param qd Quaternion input 2
 * @return The Quaternion difference
 */
arma::vec getQuatDifference(const arma::vec& q, const arma::vec& qd);

}  // namespace math
}  // namespace arl
#endif  // AUTHARL_CORE_MATH_ORIENTATION_H
