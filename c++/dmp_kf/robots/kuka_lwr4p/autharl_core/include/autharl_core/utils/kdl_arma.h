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

#ifndef AUTHARL_CORE_UTILS_KDL_ARMA_H
#define AUTHARL_CORE_UTILS_KDL_ARMA_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <armadillo>

namespace arl
{
namespace utils
{
/**
 * @brief Converts an arma vector to a KDL Vector.
 *
 * @param ros The arma vector as input
 * @param kdl The KDL vector as ouput
 */
inline void armaToKdl(const arma::vec& arma,
                      KDL::Vector* kdl)
{
  kdl->data[0] = arma(0);
  kdl->data[1] = arma(1);
  kdl->data[2] = arma(2);
}

/**
 * @brief Converts a armadillo 3D matrix to a KDL Rotation matrix.
 *
 * @param arma The arma matrix as input
 * @param kdl The KDL rotation matrix as ouput
 */
inline void armaToKdl(const arma::mat& arma,
                      KDL::Rotation* kdl)
{
  kdl->data[0] = arma(0, 0);
  kdl->data[1] = arma(0, 1);
  kdl->data[2] = arma(0, 2);
  kdl->data[3] = arma(1, 0);
  kdl->data[4] = arma(1, 1);
  kdl->data[5] = arma(1, 2);
  kdl->data[6] = arma(2, 0);
  kdl->data[7] = arma(2, 1);
  kdl->data[8] = arma(2, 2);
}

/**
 * @brief Converts an arma homogeneous transformation (4x4 matrix) to a KDL Frame.
 *
 * @param arma The arma homogeneous transformation as input
 * @param kdl The KDL Frame as ouput
 */
inline void armaToKdl(const arma::mat& arma,
                      KDL::Frame* kdl)
{
  kdl->M.data[0] = arma(0, 0);
  kdl->M.data[1] = arma(0, 1);
  kdl->M.data[2] = arma(0, 2);
  kdl->M.data[3] = arma(1, 0);
  kdl->M.data[4] = arma(1, 1);
  kdl->M.data[5] = arma(1, 2);
  kdl->M.data[6] = arma(2, 0);
  kdl->M.data[7] = arma(2, 1);
  kdl->M.data[8] = arma(2, 2);

  kdl->p.data[0] = arma(0, 3);
  kdl->p.data[1] = arma(1, 3);
  kdl->p.data[2] = arma(2, 3);
}

inline void armaToKdl(const arma::vec& arma,
                      KDL::Wrench* kdl)
{
  kdl->force.data[0] = arma(0);
  kdl->force.data[1] = arma(1);
  kdl->force.data[2] = arma(2);
  kdl->torque.data[0] = arma(3);
  kdl->torque.data[1] = arma(4);
  kdl->torque.data[2] = arma(5);
}

inline void armaToKdl(const arma::vec& arma,
                      KDL::Twist* kdl)
{
  kdl->vel.data[0] = arma(0);
  kdl->vel.data[1] = arma(1);
  kdl->vel.data[2] = arma(2);
  kdl->rot.data[0] = arma(3);
  kdl->rot.data[1] = arma(4);
  kdl->rot.data[2] = arma(5);
}

inline void armaToKdl(const arma::mat& arma,
                      KDL::Jacobian* kdl)
{
  kdl->resize(arma.n_cols);
  for (int i = 0; i < arma.n_cols; i++)
  {
    kdl->data(0, i) = arma(0, i);
    kdl->data(1, i) = arma(1, i);
    kdl->data(2, i) = arma(2, i);
    kdl->data(3, i) = arma(3, i);
    kdl->data(4, i) = arma(4, i);
    kdl->data(5, i) = arma(5, i);
  }
}


inline void armaToKdl(const arma::vec& arma,
                      KDL::JntArray* kdl)
{
  kdl->resize(arma.size());
  for (int i = 0; i < arma.size(); i++)
  {
    kdl->data[i] = arma(i);
  }
}

/**
 * @brief Converts a KDL vector to an arma vector
 *
 * @param kdl The KDL vector as input
 * @param arma The arma vector as output
 */
inline void kdlToArma(const KDL::Vector& kdl,
                      arma::vec* arma)
{
  arma->resize(3);
  for (unsigned int i = 0; i < 3; i++)
  {
    (*arma)(i) = kdl(i);
  }
}

/**
 * @brief Converts a KDL Rotation to an arma matrix.
 *
 * @param kdl The KDL Rotation as input
 * @param arma The arma matrix as output
 */
inline void kdlToArma(const KDL::Rotation& kdl,
                      arma::mat* arma)
{
  arma->resize(3, 3);
  (*arma)(0, 0) = kdl.data[0];
  (*arma)(0, 1) = kdl.data[1];
  (*arma)(0, 2) = kdl.data[2];
  (*arma)(1, 0) = kdl.data[3];
  (*arma)(1, 1) = kdl.data[4];
  (*arma)(1, 2) = kdl.data[5];
  (*arma)(2, 0) = kdl.data[6];
  (*arma)(2, 1) = kdl.data[7];
  (*arma)(2, 2) = kdl.data[8];
}

/**
 * @brief Converts a KDL Frame to an arma homogeneous transformation (4x4 matrix).
 *
 * @param kdl The KDL Frame as input
 * @param arma The arma matrix as output
 */
inline void kdlToArma(const KDL::Frame& kdl,
                      arma::mat* arma)
{
  arma->resize(3, 4);
  (*arma)(0, 0) = kdl.M.data[0];
  (*arma)(0, 1) = kdl.M.data[1];
  (*arma)(0, 2) = kdl.M.data[2];
  (*arma)(1, 0) = kdl.M.data[3];
  (*arma)(1, 1) = kdl.M.data[4];
  (*arma)(1, 2) = kdl.M.data[5];
  (*arma)(2, 0) = kdl.M.data[6];
  (*arma)(2, 1) = kdl.M.data[7];
  (*arma)(2, 2) = kdl.M.data[8];
  (*arma)(0, 3) = kdl.p.data[0];
  (*arma)(1, 3) = kdl.p.data[1];
  (*arma)(2, 3) = kdl.p.data[2];
}

inline void kdlToArma(const KDL::Wrench& kdl,
                      arma::vec* arma)
{
  arma->resize(6);
  for (unsigned int i = 0; i < 6; i++)
  {
    (*arma)(i) = kdl(i);
  }
}

inline void kdlToArma(const KDL::Twist& kdl,
                      arma::vec* arma)
{
  arma->resize(6);
  for (unsigned int i = 0; i < 6; i++)
  {
    (*arma)(i) = kdl(i);
  }
}

inline void kdlToArma(const KDL::Jacobian& kdl,
                        arma::mat* arma)
{
  arma->resize(6, kdl.columns());
  for (int i = 0; i < kdl.columns(); i++)
  {
    (*arma)(0, i) = kdl.data(0, i);
    (*arma)(1, i) = kdl.data(1, i);
    (*arma)(2, i) = kdl.data(2, i);
    (*arma)(3, i) = kdl.data(3, i);
    (*arma)(4, i) = kdl.data(4, i);
    (*arma)(5, i) = kdl.data(5, i);
  }
}

inline void kdlToArma(const KDL::JntArray& kdl,
                      arma::vec* arma)
{
  arma::vec temp(kdl.data.size());
  arma->resize(kdl.data.size());
  for (int i = 0; i < kdl.data.size(); i++)
  {
    (*arma)(i) = kdl.data[i];
  }
}

}  // namespace utils
}  // namespace arl
#endif  // AUTHARL_CORE_UTILS_KDL_ARMA_H
