//
// File: xrot.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 01-Mar-2018 15:38:30
//

// Include Files
#include "rt_nonfinite.h"
#include "mat2quat.h"
#include "xrot.h"

// Function Definitions

//
// Arguments    : int n
//                double x[16]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void b_xrot(int n, double x[16], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  if (!(n < 1)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 1; k <= n; k++) {
      temp = c * x[ix] + s * x[iy];
      x[iy] = c * x[iy] - s * x[ix];
      x[ix] = temp;
      iy++;
      ix++;
    }
  }
}

//
// Arguments    : int n
//                double x[16]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void xrot(int n, double x[16], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  if (!(n < 1)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 1; k <= n; k++) {
      temp = c * x[ix] + s * x[iy];
      x[iy] = c * x[iy] - s * x[ix];
      x[ix] = temp;
      iy += 4;
      ix += 4;
    }
  }
}

//
// File trailer for xrot.cpp
//
// [EOF]
//
