#ifndef MATH_LIB_UNIT_QUATERNIONS_64_H
#define MATH_LIB_UNIT_QUATERNIONS_64_H

#include <Eigen/Dense>
#include <armadillo>
#include <math_lib/math.h>

namespace as64_
{

namespace math_
{

Eigen::Vector4d quatInv(const Eigen::Vector4d &quat);
arma::vec quatInv(const arma::vec &quat);

Eigen::Matrix4d quat2qmat(const Eigen::Vector4d &quat);

Eigen::Vector4d quatProd(const Eigen::Vector4d &quat1, const Eigen::Vector4d &quat2);
arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2);

// If quat1 and quat2 were positions, this would perform quat1-quat2.
// The result is the amount of rotation needed to go from quat2 to quat1, i.e. quatD*quat2 = quat1
// Equivalently, the result is quaternion corresponding to the angular velocity which takes us from quat2 to quat1 in unit time.
Eigen::Vector4d quatDiff(const Eigen::Vector4d &quat1, const Eigen::Vector4d &quat2);
arma::vec quatDiff(const arma::vec &quat1, const arma::vec &quat2);

Eigen::Vector4d quatExp(const Eigen::Vector3d &v_rot, double zero_tol=1e-16);
arma::vec quatExp(const arma::vec &v_rot, double zero_tol=1e-16);

Eigen::Vector3d quatLog(const Eigen::Vector4d &quat, double zero_tol=1e-16);
arma::vec quatLog(const arma::vec &quat, double zero_tol=1e-16);

Eigen::Vector4d get_quat_dot(const Eigen::Vector3d &omega, Eigen::Vector4d &quat);

/** Converts quternion pseudo-position to unit quaternion.
 *  Converts the pseudo quaternion position 'qPos' to unit quaternion 'Q1'
 *  using as origin the unit quaternion 'Q2'.
 *  Given two unit quaternions Q1, Q2, the pseudo quaternion position of Q1 with
 *  respect to Q2 is calculated as:
 *  qPos = exp(Q1 * Q2^-1)
 *  The original unit quaternion from a pseudo quaternion position us given by:
 *  Q1 = log(qPos) * Q2
 *  @param[in] qPos 3 X 1 with the pseudo quaternion position.
 *  @param[in] Q2 4 x 1 vector containing the unit quaternion origin (optional, default = [1 0 0 0]').
 *  @param[out] Q1 4 X 1 containing the unit quaternion resulting from the pseudo position and the given origin quaternion.
 */
arma::vec qpos2quat(const arma::vec &qPos, const arma::vec &Q2);
arma::vec qpos2quat(const arma::vec &qPos);

/** Converts unit quaternion to quternion pseudo-position.
 *  Converts the unit quaternion 'Q1' to pseudo quaternion position 'qPos'
 *  using as origin the unit quaternion 'Q2'.
 *  Given two unit quaternions Q1, Q2, the pseudo quaternion position of Q1 with
 *  respect to Q2 is calculated as:
 *  qPos = exp(Q1 * Q2^-1)
 *  The original unit quaternion from a pseudo quaternion position us given by:
 *  Q1 = log(qPos) * Q2
 *  @param[in] Q2: 4 x 1 vector which is the unit quaternion origin (optional, default = [1 0 0 0]').
 *  @param[in] Q1: 4 X 1 vector with the unit quaternion to be converted.
 *  @param[out] qPos: 3 X 1 vector with the pseudo quaternion position.
 */
arma::vec quat2qpos(const arma::vec &Q1, const arma::vec &Q2);
arma::vec quat2qpos(const arma::vec &Q1);

} // namespace math_

} // namespace as64_

#endif // MATH_LIB_UNIT_QUATERNIONS_64_H
