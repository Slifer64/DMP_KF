//
// File: xzlascl.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 01-Mar-2018 15:38:30
//

// Include Files
#include "rt_nonfinite.h"
#include "mat2quat.h"
#include "xzlascl.h"

// Function Definitions

//
// Arguments    : double cfrom
//                double cto
//                creal_T A[16]
// Return Type  : void
//
void xzlascl(double cfrom, double cto, creal_T A[16])
{
  double cfromc;
  double ctoc;
  boolean_T notdone;
  double cfrom1;
  double cto1;
  double mul;
  int i6;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (i6 = 0; i6 < 16; i6++) {
      A[i6].re *= mul;
      A[i6].im *= mul;
    }
  }
}

//
// File trailer for xzlascl.cpp
//
// [EOF]
//
