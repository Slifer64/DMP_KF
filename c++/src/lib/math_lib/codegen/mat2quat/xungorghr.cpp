//
// File: xungorghr.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 01-Mar-2018 15:38:30
//

// Include Files
#include "rt_nonfinite.h"
#include "mat2quat.h"
#include "xungorghr.h"
#include "xgerc.h"

// Function Definitions

//
// Arguments    : int n
//                int ilo
//                int ihi
//                double A[16]
//                int ia0
//                const double tau[3]
//                int itau0
// Return Type  : void
//
void xungorghr(int n, int ilo, int ihi, double A[16], int ia0, const double tau
               [3], int itau0)
{
  int nh;
  int i;
  int ia;
  int b_i;
  int itau;
  int b_ia;
  double work[4];
  int iaii;
  int lastv;
  int lastc;
  int i4;
  boolean_T exitg2;
  int iac;
  int exitg1;
  int ix;
  double c;
  int i5;
  if (n != 0) {
    nh = ihi - ilo;
    for (i = ihi; i >= ilo + 1; i--) {
      ia = (ia0 + ((i - 1) << 2)) - 2;
      for (b_i = 1; b_i < i; b_i++) {
        A[ia + b_i] = 0.0;
      }

      for (b_i = i + 1; b_i <= ihi; b_i++) {
        A[ia + b_i] = A[(ia + b_i) - 4];
      }

      for (b_i = ihi + 1; b_i <= n; b_i++) {
        A[ia + b_i] = 0.0;
      }
    }

    for (i = 0; i + 1 <= ilo; i++) {
      ia = (ia0 + (i << 2)) - 1;
      for (b_i = 1; b_i <= n; b_i++) {
        A[(ia + b_i) - 1] = 0.0;
      }

      A[ia + i] = 1.0;
    }

    for (i = ihi; i + 1 <= n; i++) {
      ia = (ia0 + (i << 2)) - 1;
      for (b_i = 1; b_i <= n; b_i++) {
        A[(ia + b_i) - 1] = 0.0;
      }

      A[ia + i] = 1.0;
    }

    ia = (ia0 + ilo) + (ilo << 2);
    if (!(nh < 1)) {
      for (i = nh; i < nh; i++) {
        b_ia = ia + (i << 2);
        for (b_i = 0; b_i < nh; b_i++) {
          A[(b_ia + b_i) - 1] = 0.0;
        }

        A[(b_ia + i) - 1] = 1.0;
      }

      itau = ((itau0 + ilo) + nh) - 3;
      for (b_i = 0; b_i < 4; b_i++) {
        work[b_i] = 0.0;
      }

      for (b_i = nh; b_i >= 1; b_i--) {
        iaii = ((ia + b_i) + ((b_i - 1) << 2)) - 1;
        if (b_i < nh) {
          A[iaii - 1] = 1.0;
          i = nh - b_i;
          if (tau[itau] != 0.0) {
            lastv = i + 1;
            i += iaii;
            while ((lastv > 0) && (A[i - 1] == 0.0)) {
              lastv--;
              i--;
            }

            lastc = nh - b_i;
            exitg2 = false;
            while ((!exitg2) && (lastc > 0)) {
              i = (iaii + ((lastc - 1) << 2)) + 4;
              b_ia = i;
              do {
                exitg1 = 0;
                if (b_ia <= (i + lastv) - 1) {
                  if (A[b_ia - 1] != 0.0) {
                    exitg1 = 1;
                  } else {
                    b_ia++;
                  }
                } else {
                  lastc--;
                  exitg1 = 2;
                }
              } while (exitg1 == 0);

              if (exitg1 == 1) {
                exitg2 = true;
              }
            }
          } else {
            lastv = 0;
            lastc = 0;
          }

          if (lastv > 0) {
            if (lastc != 0) {
              for (i = 1; i <= lastc; i++) {
                work[i - 1] = 0.0;
              }

              i = 0;
              i4 = (iaii + ((lastc - 1) << 2)) + 4;
              for (iac = iaii + 4; iac <= i4; iac += 4) {
                ix = iaii;
                c = 0.0;
                i5 = (iac + lastv) - 1;
                for (b_ia = iac; b_ia <= i5; b_ia++) {
                  c += A[b_ia - 1] * A[ix - 1];
                  ix++;
                }

                work[i] += c;
                i++;
              }
            }

            xgerc(lastv, lastc, -tau[itau], iaii, work, A, iaii + 4);
          }

          i4 = (iaii + nh) - b_i;
          for (i = iaii; i + 1 <= i4; i++) {
            A[i] *= -tau[itau];
          }
        }

        A[iaii - 1] = 1.0 - tau[itau];
        for (i = 1; i < b_i; i++) {
          A[(iaii - i) - 1] = 0.0;
        }

        itau--;
      }
    }
  }
}

//
// File trailer for xungorghr.cpp
//
// [EOF]
//
