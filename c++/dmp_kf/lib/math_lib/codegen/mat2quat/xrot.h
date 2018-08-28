//
// File: xrot.h
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 01-Mar-2018 15:38:30
//
#ifndef XROT_H
#define XROT_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "mat2quat_types.h"

// Function Declarations
extern void b_xrot(int n, double x[16], int ix0, int iy0, double c, double s);
extern void xrot(int n, double x[16], int ix0, int iy0, double c, double s);

#endif

//
// File trailer for xrot.h
//
// [EOF]
//
