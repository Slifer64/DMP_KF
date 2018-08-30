//
// File: xzlanhs.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 01-Mar-2018 15:38:30
//

// Include Files
#include "rt_nonfinite.h"
#include "mat2quat.h"
#include "xzlanhs.h"

// Function Definitions

//
// Arguments    : const creal_T A[16]
//                int ilo
//                int ihi
// Return Type  : double
//
double xzlanhs(const creal_T A[16], int ilo, int ihi)
{
  double f;
  double scale;
  double sumsq;
  boolean_T firstNonZero;
  int j;
  int i0;
  int i;
  double reAij;
  double imAij;
  double temp2;
  f = 0.0;
  if (!(ilo > ihi)) {
    scale = 0.0;
    sumsq = 0.0;
    firstNonZero = true;
    for (j = ilo; j <= ihi; j++) {
      i0 = j + 1;
      if (ihi < j + 1) {
        i0 = ihi;
      }

      for (i = ilo; i <= i0; i++) {
        reAij = A[(i + ((j - 1) << 2)) - 1].re;
        imAij = A[(i + ((j - 1) << 2)) - 1].im;
        if (reAij != 0.0) {
          reAij = std::abs(reAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = reAij;
            firstNonZero = false;
          } else if (scale < reAij) {
            temp2 = scale / reAij;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            scale = reAij;
          } else {
            temp2 = reAij / scale;
            sumsq += temp2 * temp2;
          }
        }

        if (imAij != 0.0) {
          reAij = std::abs(imAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = reAij;
            firstNonZero = false;
          } else if (scale < reAij) {
            temp2 = scale / reAij;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            scale = reAij;
          } else {
            temp2 = reAij / scale;
            sumsq += temp2 * temp2;
          }
        }
      }
    }

    f = scale * std::sqrt(sumsq);
  }

  return f;
}

//
// File trailer for xzlanhs.cpp
//
// [EOF]
//
