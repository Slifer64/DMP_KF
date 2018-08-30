/*
 * File: _coder_mat2quat_api.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 01-Mar-2018 15:38:30
 */

#ifndef _CODER_MAT2QUAT_API_H
#define _CODER_MAT2QUAT_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_mat2quat_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void mat2quat(real_T R[9], real_T quat[4]);
extern void mat2quat_api(const mxArray *prhs[1], const mxArray *plhs[1]);
extern void mat2quat_atexit(void);
extern void mat2quat_initialize(void);
extern void mat2quat_terminate(void);
extern void mat2quat_xil_terminate(void);

#endif

/*
 * File trailer for _coder_mat2quat_api.h
 *
 * [EOF]
 */
