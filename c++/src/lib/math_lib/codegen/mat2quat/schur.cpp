//
// File: schur.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 01-Mar-2018 15:38:30
//

// Include Files
#include "rt_nonfinite.h"
#include "mat2quat.h"
#include "schur.h"
#include "xdlanv2.h"
#include "xdhseqr.h"
#include "xungorghr.h"
#include "xgehrd.h"
#include "mat2quat_rtwutil.h"

// Function Definitions

//
// Arguments    : const double A[16]
//                creal_T V[16]
//                creal_T T[16]
// Return Type  : void
//
void schur(const double A[16], creal_T V[16], creal_T T[16])
{
  double b_A[16];
  double tau[3];
  double z[16];
  int j;
  int m;
  double r;
  double s;
  double t1_re;
  double t1_im;
  double mu1_im;
  double rt1i;
  double mu1_re;
  double rt2i;
  double cs;
  double sn;
  memcpy(&b_A[0], &A[0], sizeof(double) << 4);
  xgehrd(b_A, tau);
  memcpy(&z[0], &b_A[0], sizeof(double) << 4);
  xungorghr(4, 1, 4, z, 1, tau, 1);
  eml_dlahqr(b_A, z);
  b_A[3] = 0.0;
  for (j = 0; j < 16; j++) {
    T[j].re = b_A[j];
    T[j].im = 0.0;
    V[j].re = z[j];
    V[j].im = 0.0;
  }

  for (m = 2; m >= 0; m += -1) {
    if (b_A[(m + (m << 2)) + 1] != 0.0) {
      r = b_A[m + (m << 2)];
      s = b_A[m + ((m + 1) << 2)];
      t1_re = b_A[(m + (m << 2)) + 1];
      t1_im = b_A[(m + ((m + 1) << 2)) + 1];
      xdlanv2(&r, &s, &t1_re, &t1_im, &mu1_im, &rt1i, &mu1_re, &rt2i, &cs, &sn);
      mu1_re = mu1_im - b_A[(m + ((m + 1) << 2)) + 1];
      r = rt_hypotd_snf(rt_hypotd_snf(mu1_re, rt1i), b_A[(m + (m << 2)) + 1]);
      if (rt1i == 0.0) {
        mu1_re /= r;
        mu1_im = 0.0;
      } else if (mu1_re == 0.0) {
        mu1_re = 0.0;
        mu1_im = rt1i / r;
      } else {
        mu1_re /= r;
        mu1_im = rt1i / r;
      }

      s = b_A[(m + (m << 2)) + 1] / r;
      for (j = m; j + 1 < 5; j++) {
        t1_re = T[m + (j << 2)].re;
        t1_im = T[m + (j << 2)].im;
        r = T[m + (j << 2)].re;
        T[m + (j << 2)].re = (mu1_re * T[m + (j << 2)].re + mu1_im * T[m + (j <<
          2)].im) + s * T[(m + (j << 2)) + 1].re;
        T[m + (j << 2)].im = (mu1_re * T[m + (j << 2)].im - mu1_im * r) + s * T
          [(m + (j << 2)) + 1].im;
        r = mu1_re * T[(m + (j << 2)) + 1].im + mu1_im * T[(m + (j << 2)) + 1].
          re;
        T[(m + (j << 2)) + 1].re = (mu1_re * T[(m + (j << 2)) + 1].re - mu1_im *
          T[(m + (j << 2)) + 1].im) - s * t1_re;
        T[(m + (j << 2)) + 1].im = r - s * t1_im;
      }

      for (j = 0; j + 1 <= m + 2; j++) {
        t1_re = T[j + (m << 2)].re;
        t1_im = T[j + (m << 2)].im;
        r = mu1_re * T[j + (m << 2)].im + mu1_im * T[j + (m << 2)].re;
        T[j + (m << 2)].re = (mu1_re * T[j + (m << 2)].re - mu1_im * T[j + (m <<
          2)].im) + s * T[j + ((m + 1) << 2)].re;
        T[j + (m << 2)].im = r + s * T[j + ((m + 1) << 2)].im;
        r = T[j + ((m + 1) << 2)].re;
        T[j + ((m + 1) << 2)].re = (mu1_re * T[j + ((m + 1) << 2)].re + mu1_im *
          T[j + ((m + 1) << 2)].im) - s * t1_re;
        T[j + ((m + 1) << 2)].im = (mu1_re * T[j + ((m + 1) << 2)].im - mu1_im *
          r) - s * t1_im;
      }

      for (j = 0; j < 4; j++) {
        t1_re = V[j + (m << 2)].re;
        t1_im = V[j + (m << 2)].im;
        r = mu1_re * V[j + (m << 2)].im + mu1_im * V[j + (m << 2)].re;
        V[j + (m << 2)].re = (mu1_re * V[j + (m << 2)].re - mu1_im * V[j + (m <<
          2)].im) + s * V[j + ((m + 1) << 2)].re;
        V[j + (m << 2)].im = r + s * V[j + ((m + 1) << 2)].im;
        r = V[j + ((m + 1) << 2)].re;
        V[j + ((m + 1) << 2)].re = (mu1_re * V[j + ((m + 1) << 2)].re + mu1_im *
          V[j + ((m + 1) << 2)].im) - s * t1_re;
        V[j + ((m + 1) << 2)].im = (mu1_re * V[j + ((m + 1) << 2)].im - mu1_im *
          r) - s * t1_im;
      }

      T[(m + (m << 2)) + 1].re = 0.0;
      T[(m + (m << 2)) + 1].im = 0.0;
    }
  }
}

//
// File trailer for schur.cpp
//
// [EOF]
//
