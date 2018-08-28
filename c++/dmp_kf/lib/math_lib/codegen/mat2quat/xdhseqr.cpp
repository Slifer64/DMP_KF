//
// File: xdhseqr.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 01-Mar-2018 15:38:30
//

// Include Files
#include "rt_nonfinite.h"
#include "mat2quat.h"
#include "xdhseqr.h"
#include "xrot.h"
#include "xdlanv2.h"
#include "xzlarfg.h"

// Function Definitions

//
// Arguments    : double h[16]
//                double z[16]
// Return Type  : int
//
int eml_dlahqr(double h[16], double z[16])
{
  int info;
  double v[3];
  int i;
  boolean_T exitg1;
  int L;
  boolean_T goto150;
  int its;
  boolean_T exitg2;
  int k;
  boolean_T exitg4;
  double tst;
  double htmp1;
  double aa;
  double ab;
  double ba;
  double rt2r;
  double rt1r;
  double s;
  double sn;
  boolean_T guard1 = false;
  int hoffset;
  int nr;
  int m;
  boolean_T exitg3;
  double d1;
  int b_k;
  int j;
  info = 0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  i = 3;
  exitg1 = false;
  while ((!exitg1) && (i + 1 >= 1)) {
    L = 1;
    goto150 = false;
    its = 0;
    exitg2 = false;
    while ((!exitg2) && (its < 31)) {
      k = i;
      exitg4 = false;
      while ((!exitg4) && ((k + 1 > L) && (!(std::abs(h[k + ((k - 1) << 2)]) <=
                4.0083367200179456E-292)))) {
        tst = std::abs(h[(k + ((k - 1) << 2)) - 1]) + std::abs(h[k + (k << 2)]);
        if (tst == 0.0) {
          if (k - 1 >= 1) {
            tst = std::abs(h[(k + ((k - 2) << 2)) - 1]);
          }

          if (k + 2 <= 4) {
            tst += std::abs(h[(k + (k << 2)) + 1]);
          }
        }

        guard1 = false;
        if (std::abs(h[k + ((k - 1) << 2)]) <= 2.2204460492503131E-16 * tst) {
          htmp1 = std::abs(h[k + ((k - 1) << 2)]);
          tst = std::abs(h[(k + (k << 2)) - 1]);
          if (htmp1 > tst) {
            ab = htmp1;
            ba = tst;
          } else {
            ab = tst;
            ba = htmp1;
          }

          htmp1 = std::abs(h[k + (k << 2)]);
          tst = std::abs(h[(k + ((k - 1) << 2)) - 1] - h[k + (k << 2)]);
          if (htmp1 > tst) {
            aa = htmp1;
            htmp1 = tst;
          } else {
            aa = tst;
          }

          s = aa + ab;
          tst = 2.2204460492503131E-16 * (htmp1 * (aa / s));
          if ((4.0083367200179456E-292 >= tst) || rtIsNaN(tst)) {
            d1 = 4.0083367200179456E-292;
          } else {
            d1 = tst;
          }

          if (ba * (ab / s) <= d1) {
            exitg4 = true;
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          k--;
        }
      }

      L = k + 1;
      if (k + 1 > 1) {
        h[k + ((k - 1) << 2)] = 0.0;
      }

      if (k + 1 >= i) {
        goto150 = true;
        exitg2 = true;
      } else {
        if (its == 10) {
          s = std::abs(h[(k + (k << 2)) + 1]) + std::abs(h[(k + ((k + 1) << 2))
            + 2]);
          tst = 0.75 * s + h[k + (k << 2)];
          aa = -0.4375 * s;
          htmp1 = s;
          ba = tst;
        } else if (its == 20) {
          s = std::abs(h[i + ((i - 1) << 2)]) + std::abs(h[(i + ((i - 2) << 2))
            - 1]);
          tst = 0.75 * s + h[i + (i << 2)];
          aa = -0.4375 * s;
          htmp1 = s;
          ba = tst;
        } else {
          tst = h[(i + ((i - 1) << 2)) - 1];
          htmp1 = h[i + ((i - 1) << 2)];
          aa = h[(i + (i << 2)) - 1];
          ba = h[i + (i << 2)];
        }

        s = ((std::abs(tst) + std::abs(aa)) + std::abs(htmp1)) + std::abs(ba);
        if (s == 0.0) {
          rt1r = 0.0;
          ab = 0.0;
          rt2r = 0.0;
          ba = 0.0;
        } else {
          tst /= s;
          htmp1 /= s;
          aa /= s;
          ba /= s;
          ab = (tst + ba) / 2.0;
          tst = (tst - ab) * (ba - ab) - aa * htmp1;
          htmp1 = std::sqrt(std::abs(tst));
          if (tst >= 0.0) {
            rt1r = ab * s;
            rt2r = rt1r;
            ab = htmp1 * s;
            ba = -ab;
          } else {
            rt1r = ab + htmp1;
            rt2r = ab - htmp1;
            if (std::abs(rt1r - ba) <= std::abs(rt2r - ba)) {
              rt1r *= s;
              rt2r = rt1r;
            } else {
              rt2r *= s;
              rt1r = rt2r;
            }

            ab = 0.0;
            ba = 0.0;
          }
        }

        m = i - 1;
        exitg3 = false;
        while ((!exitg3) && (m >= k + 1)) {
          s = (std::abs(h[(m + ((m - 1) << 2)) - 1] - rt2r) + std::abs(ba)) +
            std::abs(h[m + ((m - 1) << 2)]);
          tst = h[m + ((m - 1) << 2)] / s;
          v[0] = (tst * h[(m + (m << 2)) - 1] + (h[(m + ((m - 1) << 2)) - 1] -
                   rt1r) * ((h[(m + ((m - 1) << 2)) - 1] - rt2r) / s)) - ab *
            (ba / s);
          v[1] = tst * (((h[(m + ((m - 1) << 2)) - 1] + h[m + (m << 2)]) - rt1r)
                        - rt2r);
          v[2] = tst * h[(m + (m << 2)) + 1];
          s = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
          tst = v[0] / s;
          v[0] /= s;
          htmp1 = v[1] / s;
          v[1] /= s;
          aa = v[2] / s;
          v[2] /= s;
          if ((m == k + 1) || (std::abs(h[(m + ((m - 2) << 2)) - 1]) * (std::abs
                (htmp1) + std::abs(aa)) <= 2.2204460492503131E-16 * std::abs(tst)
                               * ((std::abs(h[(m + ((m - 2) << 2)) - 2]) + std::
                 abs(h[(m + ((m - 1) << 2)) - 1])) + std::abs(h[m + (m << 2)]))))
          {
            exitg3 = true;
          } else {
            m--;
          }
        }

        for (b_k = m; b_k <= i; b_k++) {
          nr = (i - b_k) + 2;
          if (3 <= nr) {
            nr = 3;
          }

          if (b_k > m) {
            hoffset = b_k + ((b_k - 2) << 2);
            for (j = -1; j + 2 <= nr; j++) {
              v[j + 1] = h[j + hoffset];
            }
          }

          tst = v[0];
          rt2r = xzlarfg(nr, &tst, v);
          v[0] = tst;
          if (b_k > m) {
            h[(b_k + ((b_k - 2) << 2)) - 1] = tst;
            h[b_k + ((b_k - 2) << 2)] = 0.0;
            if (b_k < i) {
              h[b_k + 1] = 0.0;
            }
          } else {
            if (m > k + 1) {
              h[b_k - 1] *= 1.0 - rt2r;
            }
          }

          tst = v[1];
          htmp1 = rt2r * v[1];
          if (nr == 3) {
            ab = v[2];
            ba = rt2r * v[2];
            for (j = b_k - 1; j + 1 < 5; j++) {
              aa = (h[(b_k + (j << 2)) - 1] + tst * h[b_k + (j << 2)]) + ab * h
                [(b_k + (j << 2)) + 1];
              h[(b_k + (j << 2)) - 1] -= aa * rt2r;
              h[b_k + (j << 2)] -= aa * htmp1;
              h[(b_k + (j << 2)) + 1] -= aa * ba;
            }

            if (b_k + 3 <= i + 1) {
              nr = b_k;
            } else {
              nr = i - 2;
            }

            for (j = 0; j + 1 <= nr + 3; j++) {
              aa = (h[j + ((b_k - 1) << 2)] + tst * h[j + (b_k << 2)]) + ab *
                h[j + ((b_k + 1) << 2)];
              h[j + ((b_k - 1) << 2)] -= aa * rt2r;
              h[j + (b_k << 2)] -= aa * htmp1;
              h[j + ((b_k + 1) << 2)] -= aa * ba;
            }

            for (j = 0; j < 4; j++) {
              aa = (z[j + ((b_k - 1) << 2)] + tst * z[j + (b_k << 2)]) + ab *
                z[j + ((b_k + 1) << 2)];
              z[j + ((b_k - 1) << 2)] -= aa * rt2r;
              z[j + (b_k << 2)] -= aa * htmp1;
              z[j + ((b_k + 1) << 2)] -= aa * ba;
            }
          } else {
            if (nr == 2) {
              for (j = b_k - 1; j + 1 < 5; j++) {
                aa = h[(b_k + (j << 2)) - 1] + tst * h[b_k + (j << 2)];
                h[(b_k + (j << 2)) - 1] -= aa * rt2r;
                h[b_k + (j << 2)] -= aa * htmp1;
              }

              for (j = 0; j + 1 <= i + 1; j++) {
                aa = h[j + ((b_k - 1) << 2)] + tst * h[j + (b_k << 2)];
                h[j + ((b_k - 1) << 2)] -= aa * rt2r;
                h[j + (b_k << 2)] -= aa * htmp1;
              }

              for (j = 0; j < 4; j++) {
                aa = z[j + ((b_k - 1) << 2)] + tst * z[j + (b_k << 2)];
                z[j + ((b_k - 1) << 2)] -= aa * rt2r;
                z[j + (b_k << 2)] -= aa * htmp1;
              }
            }
          }
        }

        its++;
      }
    }

    if (!goto150) {
      info = i + 1;
      exitg1 = true;
    } else {
      if ((L != i + 1) && (L == i)) {
        tst = h[(i + (i << 2)) - 1];
        htmp1 = h[i + ((i - 1) << 2)];
        aa = h[i + (i << 2)];
        xdlanv2(&h[(i + ((i - 1) << 2)) - 1], &tst, &htmp1, &aa, &ab, &ba, &rt2r,
                &rt1r, &s, &sn);
        h[(i + (i << 2)) - 1] = tst;
        h[i + ((i - 1) << 2)] = htmp1;
        h[i + (i << 2)] = aa;
        if (4 > i + 1) {
          xrot(3 - i, h, i + ((i + 1) << 2), (i + ((i + 1) << 2)) + 1, s, sn);
        }

        b_xrot(i - 1, h, 1 + ((i - 1) << 2), 1 + (i << 2), s, sn);
        hoffset = (i - 1) << 2;
        nr = i << 2;
        for (k = 0; k < 4; k++) {
          tst = s * z[hoffset] + sn * z[nr];
          z[nr] = s * z[nr] - sn * z[hoffset];
          z[hoffset] = tst;
          nr++;
          hoffset++;
        }
      }

      i = L - 2;
    }
  }

  return info;
}

//
// File trailer for xdhseqr.cpp
//
// [EOF]
//
