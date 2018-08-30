//
// File: sqrt.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 01-Mar-2018 15:38:30
//

// Include Files
#include "rt_nonfinite.h"
#include "mat2quat.h"
#include "sqrt.h"
#include "schur.h"
#include "mat2quat_rtwutil.h"

// Function Definitions

//
// Arguments    : creal_T *x
// Return Type  : void
//
void b_sqrt(creal_T *x)
{
  double absxi;
  double absxr;
  if (x->im == 0.0) {
    if (x->re < 0.0) {
      absxi = 0.0;
      absxr = std::sqrt(std::abs(x->re));
    } else {
      absxi = std::sqrt(x->re);
      absxr = 0.0;
    }
  } else if (x->re == 0.0) {
    if (x->im < 0.0) {
      absxi = std::sqrt(-x->im / 2.0);
      absxr = -absxi;
    } else {
      absxi = std::sqrt(x->im / 2.0);
      absxr = absxi;
    }
  } else if (rtIsNaN(x->re) || rtIsNaN(x->im)) {
    absxi = rtNaN;
    absxr = rtNaN;
  } else if (rtIsInf(x->im)) {
    absxi = rtInf;
    absxr = x->im;
  } else if (rtIsInf(x->re)) {
    if (x->re < 0.0) {
      absxi = 0.0;
      absxr = rtInf;
    } else {
      absxi = rtInf;
      absxr = 0.0;
    }
  } else {
    absxr = std::abs(x->re);
    absxi = std::abs(x->im);
    if ((absxr > 4.4942328371557893E+307) || (absxi > 4.4942328371557893E+307))
    {
      absxr *= 0.5;
      absxi *= 0.5;
      absxi = rt_hypotd_snf(absxr, absxi);
      if (absxi > absxr) {
        absxi = std::sqrt(absxi) * std::sqrt(1.0 + absxr / absxi);
      } else {
        absxi = std::sqrt(absxi) * 1.4142135623730951;
      }
    } else {
      absxi = std::sqrt((rt_hypotd_snf(absxr, absxi) + absxr) * 0.5);
    }

    if (x->re > 0.0) {
      absxr = 0.5 * (x->im / absxi);
    } else {
      if (x->im < 0.0) {
        absxr = -absxi;
      } else {
        absxr = absxi;
      }

      absxi = 0.5 * (x->im / absxr);
    }
  }

  x->re = absxi;
  x->im = absxr;
}

//
// File trailer for sqrt.cpp
//
// [EOF]
//
