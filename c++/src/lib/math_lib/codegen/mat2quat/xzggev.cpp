//
// File: xzggev.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 01-Mar-2018 15:38:30
//

// Include Files
#include "rt_nonfinite.h"
#include "mat2quat.h"
#include "xzggev.h"
#include "xzlartg.h"
#include "xztgevc.h"
#include "xzhgeqz.h"
#include "xzlascl.h"
#include "schur.h"
#include "mat2quat_rtwutil.h"

// Function Definitions

//
// Arguments    : creal_T A[16]
//                int *info
//                creal_T alpha1[4]
//                creal_T beta1[4]
//                creal_T V[16]
// Return Type  : void
//
void xzggev(creal_T A[16], int *info, creal_T alpha1[4], creal_T beta1[4],
            creal_T V[16])
{
  double anrm;
  int nzcount;
  boolean_T exitg7;
  double absxk;
  boolean_T ilascl;
  double anrmto;
  int i;
  int rscale[4];
  int ilo;
  int ihi;
  int exitg2;
  int j;
  boolean_T found;
  int ii;
  boolean_T exitg5;
  int jj;
  creal_T atmp;
  boolean_T exitg6;
  int exitg1;
  signed char I[16];
  boolean_T guard2 = false;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T guard1 = false;
  double mul;
  double stemp_re;
  double cto1;
  double stemp_im;
  *info = 0;
  anrm = 0.0;
  nzcount = 0;
  exitg7 = false;
  while ((!exitg7) && (nzcount < 16)) {
    absxk = rt_hypotd_snf(A[nzcount].re, A[nzcount].im);
    if (rtIsNaN(absxk)) {
      anrm = rtNaN;
      exitg7 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }

      nzcount++;
    }
  }

  if (!((!rtIsInf(anrm)) && (!rtIsNaN(anrm)))) {
    for (i = 0; i < 4; i++) {
      alpha1[i].re = rtNaN;
      alpha1[i].im = 0.0;
      beta1[i].re = rtNaN;
      beta1[i].im = 0.0;
    }

    for (nzcount = 0; nzcount < 16; nzcount++) {
      V[nzcount].re = rtNaN;
      V[nzcount].im = 0.0;
    }
  } else {
    ilascl = false;
    anrmto = anrm;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      anrmto = 6.7178761075670888E-139;
      ilascl = true;
    } else {
      if (anrm > 1.4885657073574029E+138) {
        anrmto = 1.4885657073574029E+138;
        ilascl = true;
      }
    }

    if (ilascl) {
      xzlascl(anrm, anrmto, A);
    }

    for (i = 0; i < 4; i++) {
      rscale[i] = 1;
    }

    ilo = 0;
    ihi = 4;
    do {
      exitg2 = 0;
      i = 0;
      j = 0;
      found = false;
      ii = ihi;
      exitg5 = false;
      while ((!exitg5) && (ii > 0)) {
        nzcount = 0;
        i = ii;
        j = ihi;
        jj = 1;
        exitg6 = false;
        while ((!exitg6) && (jj <= ihi)) {
          guard2 = false;
          if ((A[(ii + ((jj - 1) << 2)) - 1].re != 0.0) || (A[(ii + ((jj - 1) <<
                 2)) - 1].im != 0.0) || (ii == jj)) {
            if (nzcount == 0) {
              j = jj;
              nzcount = 1;
              guard2 = true;
            } else {
              nzcount = 2;
              exitg6 = true;
            }
          } else {
            guard2 = true;
          }

          if (guard2) {
            jj++;
          }
        }

        if (nzcount < 2) {
          found = true;
          exitg5 = true;
        } else {
          ii--;
        }
      }

      if (!found) {
        exitg2 = 2;
      } else {
        if (i != ihi) {
          for (nzcount = 0; nzcount < 4; nzcount++) {
            atmp = A[(i + (nzcount << 2)) - 1];
            A[(i + (nzcount << 2)) - 1] = A[(ihi + (nzcount << 2)) - 1];
            A[(ihi + (nzcount << 2)) - 1] = atmp;
          }
        }

        if (j != ihi) {
          for (nzcount = 0; nzcount + 1 <= ihi; nzcount++) {
            atmp = A[nzcount + ((j - 1) << 2)];
            A[nzcount + ((j - 1) << 2)] = A[nzcount + ((ihi - 1) << 2)];
            A[nzcount + ((ihi - 1) << 2)] = atmp;
          }
        }

        rscale[ihi - 1] = j;
        ihi--;
        if (ihi == 1) {
          rscale[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        i = 0;
        j = 0;
        found = false;
        jj = ilo + 1;
        exitg3 = false;
        while ((!exitg3) && (jj <= ihi)) {
          nzcount = 0;
          i = ihi;
          j = jj;
          ii = ilo + 1;
          exitg4 = false;
          while ((!exitg4) && (ii <= ihi)) {
            guard1 = false;
            if ((A[(ii + ((jj - 1) << 2)) - 1].re != 0.0) || (A[(ii + ((jj - 1) <<
                   2)) - 1].im != 0.0) || (ii == jj)) {
              if (nzcount == 0) {
                i = ii;
                nzcount = 1;
                guard1 = true;
              } else {
                nzcount = 2;
                exitg4 = true;
              }
            } else {
              guard1 = true;
            }

            if (guard1) {
              ii++;
            }
          }

          if (nzcount < 2) {
            found = true;
            exitg3 = true;
          } else {
            jj++;
          }
        }

        if (!found) {
          exitg1 = 1;
        } else {
          if (i != ilo + 1) {
            for (nzcount = ilo; nzcount + 1 < 5; nzcount++) {
              atmp = A[(i + (nzcount << 2)) - 1];
              A[(i + (nzcount << 2)) - 1] = A[ilo + (nzcount << 2)];
              A[ilo + (nzcount << 2)] = atmp;
            }
          }

          if (j != ilo + 1) {
            for (nzcount = 0; nzcount + 1 <= ihi; nzcount++) {
              atmp = A[nzcount + ((j - 1) << 2)];
              A[nzcount + ((j - 1) << 2)] = A[nzcount + (ilo << 2)];
              A[nzcount + (ilo << 2)] = atmp;
            }
          }

          rscale[ilo] = j;
          ilo++;
          if (ilo + 1 == ihi) {
            rscale[ilo] = ilo + 1;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }

    for (nzcount = 0; nzcount < 16; nzcount++) {
      I[nzcount] = 0;
    }

    for (nzcount = 0; nzcount < 4; nzcount++) {
      I[nzcount + (nzcount << 2)] = 1;
    }

    for (nzcount = 0; nzcount < 16; nzcount++) {
      V[nzcount].re = I[nzcount];
      V[nzcount].im = 0.0;
    }

    if (!(ihi < ilo + 3)) {
      for (ii = ilo; ii + 1 < ihi - 1; ii++) {
        for (nzcount = ihi - 1; nzcount + 1 > ii + 2; nzcount--) {
          xzlartg(A[(nzcount + (ii << 2)) - 1], A[nzcount + (ii << 2)], &mul,
                  &atmp, &A[(nzcount + (ii << 2)) - 1]);
          A[nzcount + (ii << 2)].re = 0.0;
          A[nzcount + (ii << 2)].im = 0.0;
          for (j = ii + 1; j + 1 < 5; j++) {
            stemp_re = mul * A[(nzcount + (j << 2)) - 1].re + (atmp.re *
              A[nzcount + (j << 2)].re - atmp.im * A[nzcount + (j << 2)].im);
            stemp_im = mul * A[(nzcount + (j << 2)) - 1].im + (atmp.re *
              A[nzcount + (j << 2)].im + atmp.im * A[nzcount + (j << 2)].re);
            absxk = A[(nzcount + (j << 2)) - 1].im;
            cto1 = A[(nzcount + (j << 2)) - 1].re;
            A[nzcount + (j << 2)].re = mul * A[nzcount + (j << 2)].re - (atmp.re
              * A[(nzcount + (j << 2)) - 1].re + atmp.im * A[(nzcount + (j << 2))
              - 1].im);
            A[nzcount + (j << 2)].im = mul * A[nzcount + (j << 2)].im - (atmp.re
              * absxk - atmp.im * cto1);
            A[(nzcount + (j << 2)) - 1].re = stemp_re;
            A[(nzcount + (j << 2)) - 1].im = stemp_im;
          }

          atmp.re = -atmp.re;
          atmp.im = -atmp.im;
          for (i = 0; i + 1 <= ihi; i++) {
            stemp_re = mul * A[i + (nzcount << 2)].re + (atmp.re * A[i +
              ((nzcount - 1) << 2)].re - atmp.im * A[i + ((nzcount - 1) << 2)].
              im);
            stemp_im = mul * A[i + (nzcount << 2)].im + (atmp.re * A[i +
              ((nzcount - 1) << 2)].im + atmp.im * A[i + ((nzcount - 1) << 2)].
              re);
            absxk = A[i + (nzcount << 2)].im;
            cto1 = A[i + (nzcount << 2)].re;
            A[i + ((nzcount - 1) << 2)].re = mul * A[i + ((nzcount - 1) << 2)].
              re - (atmp.re * A[i + (nzcount << 2)].re + atmp.im * A[i +
                    (nzcount << 2)].im);
            A[i + ((nzcount - 1) << 2)].im = mul * A[i + ((nzcount - 1) << 2)].
              im - (atmp.re * absxk - atmp.im * cto1);
            A[i + (nzcount << 2)].re = stemp_re;
            A[i + (nzcount << 2)].im = stemp_im;
          }

          for (i = 0; i < 4; i++) {
            stemp_re = mul * V[i + (nzcount << 2)].re + (atmp.re * V[i +
              ((nzcount - 1) << 2)].re - atmp.im * V[i + ((nzcount - 1) << 2)].
              im);
            stemp_im = mul * V[i + (nzcount << 2)].im + (atmp.re * V[i +
              ((nzcount - 1) << 2)].im + atmp.im * V[i + ((nzcount - 1) << 2)].
              re);
            absxk = V[i + (nzcount << 2)].re;
            V[i + ((nzcount - 1) << 2)].re = mul * V[i + ((nzcount - 1) << 2)].
              re - (atmp.re * V[i + (nzcount << 2)].re + atmp.im * V[i +
                    (nzcount << 2)].im);
            V[i + ((nzcount - 1) << 2)].im = mul * V[i + ((nzcount - 1) << 2)].
              im - (atmp.re * V[i + (nzcount << 2)].im - atmp.im * absxk);
            V[i + (nzcount << 2)].re = stemp_re;
            V[i + (nzcount << 2)].im = stemp_im;
          }
        }
      }
    }

    xzhgeqz(A, ilo + 1, ihi, V, info, alpha1, beta1);
    if (*info == 0) {
      xztgevc(A, V);
      if (ilo + 1 > 1) {
        for (i = ilo - 1; i + 1 >= 1; i--) {
          nzcount = rscale[i] - 1;
          if (rscale[i] != i + 1) {
            for (j = 0; j < 4; j++) {
              atmp = V[i + (j << 2)];
              V[i + (j << 2)] = V[nzcount + (j << 2)];
              V[nzcount + (j << 2)] = atmp;
            }
          }
        }
      }

      if (ihi < 4) {
        while (ihi + 1 < 5) {
          nzcount = rscale[ihi] - 1;
          if (rscale[ihi] != ihi + 1) {
            for (j = 0; j < 4; j++) {
              atmp = V[ihi + (j << 2)];
              V[ihi + (j << 2)] = V[nzcount + (j << 2)];
              V[nzcount + (j << 2)] = atmp;
            }
          }

          ihi++;
        }
      }

      for (nzcount = 0; nzcount < 4; nzcount++) {
        absxk = std::abs(V[nzcount << 2].re) + std::abs(V[nzcount << 2].im);
        for (ii = 0; ii < 3; ii++) {
          cto1 = std::abs(V[(ii + (nzcount << 2)) + 1].re) + std::abs(V[(ii +
            (nzcount << 2)) + 1].im);
          if (cto1 > absxk) {
            absxk = cto1;
          }
        }

        if (absxk >= 6.7178761075670888E-139) {
          absxk = 1.0 / absxk;
          for (ii = 0; ii < 4; ii++) {
            V[ii + (nzcount << 2)].re *= absxk;
            V[ii + (nzcount << 2)].im *= absxk;
          }
        }
      }

      if (ilascl) {
        ilascl = true;
        while (ilascl) {
          absxk = anrmto * 2.0041683600089728E-292;
          cto1 = anrm / 4.9896007738368E+291;
          if ((absxk > anrm) && (anrm != 0.0)) {
            mul = 2.0041683600089728E-292;
            anrmto = absxk;
          } else if (cto1 > anrmto) {
            mul = 4.9896007738368E+291;
            anrm = cto1;
          } else {
            mul = anrm / anrmto;
            ilascl = false;
          }

          for (nzcount = 0; nzcount < 4; nzcount++) {
            alpha1[nzcount].re *= mul;
            alpha1[nzcount].im *= mul;
          }
        }
      }
    }
  }
}

//
// File trailer for xzggev.cpp
//
// [EOF]
//
