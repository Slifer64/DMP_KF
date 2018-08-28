//
// File: mat2quat.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 01-Mar-2018 15:38:30
//

// Include Files
#include "rt_nonfinite.h"
#include "mat2quat.h"
#include "schur.h"
#include "xzggev.h"

// Function Definitions

//
// ROTM2QUAT Convert rotation matrix to quaternion
//    Q = ROTM2QUAT(R) converts a 3D rotation matrix, R, into the corresponding
//    unit quaternion representation, Q. The input, R, is an 3-by-3-by-N matrix
//    containing N orthonormal rotation matrices.
//    The output, Q, is an N-by-4 matrix containing N quaternions. Each
//    quaternion is of the form q = [w x y z], with a scalar number as
//    the first value. Each element of Q must be a real number.
//
//    If the input matrices are not orthonormal, the function will
//    return the quaternions that correspond to the orthonormal matrices
//    closest to the imprecise matrix inputs.
//
//
//    Example:
//       % Convert a rotation matrix to a quaternion
//       R = [0 0 1; 0 1 0; -1 0 0];
//       q = rotm2quat(R)
//
//    References:
//    [1] I.Y. Bar-Itzhack, "New method for extracting the quaternion from a
//        rotation matrix," Journal of Guidance, Control, and Dynamics,
//        vol. 23, no. 6, pp. 1085-1087, 2000
//
//    See also quat2rotm
// Arguments    : const double R[9]
//                double quat[4]
// Return Type  : void
//

namespace matlab_codegen
{

void mat2quat(const double R[9], double quat[4])
{
  double K12;
  double K13;
  double K14;
  double K23;
  double K24;
  double K34;
  double b_R[16];
  double K[16];
  int ixstart;
  int j;
  boolean_T p;
  boolean_T exitg3;
  creal_T At[16];
  creal_T V[16];
  creal_T alpha1[4];
  int exitg2;
  creal_T beta1[4];
  int coltop;
  double varargin_1[4];
  boolean_T exitg1;

  //    Copyright 2014-2016 The MathWorks, Inc.
  //  Calculate all elements of symmetric K matrix
  K12 = R[3] + R[1];
  K13 = R[6] + R[2];
  K14 = R[5] - R[7];
  K23 = R[7] + R[5];
  K24 = R[6] - R[2];
  K34 = R[1] - R[3];

  //  Construct K matrix according to paper
  b_R[0] = (R[0] - R[4]) - R[8];
  b_R[4] = K12;
  b_R[8] = K13;
  b_R[12] = K14;
  b_R[1] = K12;
  b_R[5] = (R[4] - R[0]) - R[8];
  b_R[9] = K23;
  b_R[13] = K24;
  b_R[2] = K13;
  b_R[6] = K23;
  b_R[10] = (R[8] - R[0]) - R[4];
  b_R[14] = K34;
  b_R[3] = K14;
  b_R[7] = K24;
  b_R[11] = K34;
  b_R[15] = (R[0] + R[4]) + R[8];
  for (ixstart = 0; ixstart < 4; ixstart++) {
    for (j = 0; j < 4; j++) {
      K[j + (ixstart << 2)] = b_R[j + (ixstart << 2)] / 3.0;
    }
  }

  //  For each input rotation matrix, calculate the corresponding eigenvalues
  //  and eigenvectors. The eigenvector corresponding to the largest eigenvalue
  //  is the unit quaternion representing the same rotation.
  p = true;
  j = 0;
  exitg3 = false;
  while ((!exitg3) && (j < 4)) {
    ixstart = 0;
    do {
      exitg2 = 0;
      if (ixstart <= j) {
        if (!(K[ixstart + (j << 2)] == K[j + (ixstart << 2)])) {
          p = false;
          exitg2 = 1;
        } else {
          ixstart++;
        }
      } else {
        j++;
        exitg2 = 2;
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
      exitg3 = true;
    }
  }

  if (p) {
    schur(K, V, At);
    for (ixstart = 0; ixstart < 4; ixstart++) {
      alpha1[ixstart] = At[ixstart + (ixstart << 2)];
    }
  } else {
    for (ixstart = 0; ixstart < 16; ixstart++) {
      At[ixstart].re = K[ixstart];
      At[ixstart].im = 0.0;
    }

    xzggev(At, &ixstart, alpha1, beta1, V);
    for (coltop = 0; coltop <= 13; coltop += 4) {
      K12 = 0.0;
      K13 = 2.2250738585072014E-308;
      for (ixstart = coltop; ixstart + 1 <= coltop + 4; ixstart++) {
        K14 = std::abs(V[ixstart].re);
        if (K14 > K13) {
          K23 = K13 / K14;
          K12 = 1.0 + K12 * K23 * K23;
          K13 = K14;
        } else {
          K23 = K14 / K13;
          K12 += K23 * K23;
        }

        K14 = std::abs(V[ixstart].im);
        if (K14 > K13) {
          K23 = K13 / K14;
          K12 = 1.0 + K12 * K23 * K23;
          K13 = K14;
        } else {
          K23 = K14 / K13;
          K12 += K23 * K23;
        }
      }

      K12 = K13 * std::sqrt(K12);
      for (j = coltop; j + 1 <= coltop + 4; j++) {
        if (V[j].im == 0.0) {
          V[j].re /= K12;
          V[j].im = 0.0;
        } else if (V[j].re == 0.0) {
          V[j].re = 0.0;
          V[j].im /= K12;
        } else {
          V[j].re /= K12;
          V[j].im /= K12;
        }
      }
    }

    for (ixstart = 0; ixstart < 4; ixstart++) {
      K23 = alpha1[ixstart].re;
      if (beta1[ixstart].im == 0.0) {
        if (alpha1[ixstart].im == 0.0) {
          alpha1[ixstart].re /= beta1[ixstart].re;
          alpha1[ixstart].im = 0.0;
        } else if (alpha1[ixstart].re == 0.0) {
          alpha1[ixstart].re = 0.0;
          alpha1[ixstart].im /= beta1[ixstart].re;
        } else {
          alpha1[ixstart].re /= beta1[ixstart].re;
          alpha1[ixstart].im /= beta1[ixstart].re;
        }
      } else if (beta1[ixstart].re == 0.0) {
        if (alpha1[ixstart].re == 0.0) {
          alpha1[ixstart].re = alpha1[ixstart].im / beta1[ixstart].im;
          alpha1[ixstart].im = 0.0;
        } else if (alpha1[ixstart].im == 0.0) {
          alpha1[ixstart].re = 0.0;
          alpha1[ixstart].im = -(K23 / beta1[ixstart].im);
        } else {
          alpha1[ixstart].re = alpha1[ixstart].im / beta1[ixstart].im;
          alpha1[ixstart].im = -(K23 / beta1[ixstart].im);
        }
      } else {
        K14 = std::abs(beta1[ixstart].re);
        K12 = std::abs(beta1[ixstart].im);
        if (K14 > K12) {
          K12 = beta1[ixstart].im / beta1[ixstart].re;
          K13 = beta1[ixstart].re + K12 * beta1[ixstart].im;
          alpha1[ixstart].re = (alpha1[ixstart].re + K12 * alpha1[ixstart].im) /
            K13;
          alpha1[ixstart].im = (alpha1[ixstart].im - K12 * K23) / K13;
        } else if (K12 == K14) {
          if (beta1[ixstart].re > 0.0) {
            K12 = 0.5;
          } else {
            K12 = -0.5;
          }

          if (beta1[ixstart].im > 0.0) {
            K13 = 0.5;
          } else {
            K13 = -0.5;
          }

          alpha1[ixstart].re = (alpha1[ixstart].re * K12 + alpha1[ixstart].im *
                                K13) / K14;
          alpha1[ixstart].im = (alpha1[ixstart].im * K12 - K23 * K13) / K14;
        } else {
          K12 = beta1[ixstart].re / beta1[ixstart].im;
          K13 = beta1[ixstart].im + K12 * beta1[ixstart].re;
          alpha1[ixstart].re = (K12 * alpha1[ixstart].re + alpha1[ixstart].im) /
            K13;
          alpha1[ixstart].im = (K12 * alpha1[ixstart].im - K23) / K13;
        }
      }
    }
  }

  for (ixstart = 0; ixstart < 4; ixstart++) {
    varargin_1[ixstart] = alpha1[ixstart].re;
  }

  ixstart = 1;
  K12 = varargin_1[0];
  j = 0;
  if (rtIsNaN(varargin_1[0])) {
    coltop = 2;
    exitg1 = false;
    while ((!exitg1) && (coltop < 5)) {
      ixstart = coltop;
      if (!rtIsNaN(varargin_1[coltop - 1])) {
        K12 = varargin_1[coltop - 1];
        j = coltop - 1;
        exitg1 = true;
      } else {
        coltop++;
      }
    }
  }

  if (ixstart < 4) {
    while (ixstart + 1 < 5) {
      if (varargin_1[ixstart] > K12) {
        K12 = varargin_1[ixstart];
        j = ixstart;
      }

      ixstart++;
    }
  }

  quat[0] = V[3 + (j << 2)].re;
  quat[1] = V[j << 2].re;
  quat[2] = V[1 + (j << 2)].re;
  quat[3] = V[2 + (j << 2)].re;

  //  By convention, always keep scalar quaternion element positive.
  //  Note that this does not change the rotation that is represented
  //  by the unit quaternion, since q and -q denote the same rotation.
  if (V[3 + (j << 2)].re < 0.0) {
    for (ixstart = 0; ixstart < 4; ixstart++) {
      quat[ixstart] = -quat[ixstart];
    }
  }
}

} // namespace matlab_codegen

//
// File trailer for mat2quat.cpp
//
// [EOF]
//
