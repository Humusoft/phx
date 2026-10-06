// Copyright 2026 HUMUSOFT s.r.o.
// SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
// Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
//
// MEX build of phx.internal.computeBuoyancyXY (the .m file is the fallback).
// Build from phx/+phx/+internal:
//   mex -R2018a -O mex_sources/computeBuoyancyXY.cpp
//
#include "mex.h"

static bool isRealDouble(const mxArray* a)
{
    return mxIsDouble(a) && !mxIsComplex(a) && !mxIsSparse(a);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    if (nrhs != 3) {
        mexErrMsgIdAndTxt("phx:Buoyancy:invalidInputs", "Three inputs required.");
    }
    for (int i = 0; i < 3; ++i) {
        if (!isRealDouble(prhs[i])) {
            mexErrMsgIdAndTxt("phx:Buoyancy:invalidInputs", "All inputs must be real double arrays.");
        }
    }
    const mwSize n = mxGetNumberOfElements(prhs[2]);
    const mwSize T = mxGetN(prhs[1]);
    if (mxGetM(prhs[0]) != 16 || mxGetN(prhs[0]) != n || mxGetM(prhs[1]) != 3) {
        mexErrMsgIdAndTxt("phx:Buoyancy:invalidSize", "M must be 16xN and P 3xT for N counts.");
    }
    const double* M = mxGetDoubles(prhs[0]);
    const double* P = mxGetDoubles(prhs[1]);
    const double* counts = mxGetDoubles(prhs[2]);

    mwSize total = 0;
    for (mwSize a = 0; a < n; ++a) {
        total += static_cast<mwSize>(counts[a]);
    }
    if (total != T) {
        mexErrMsgIdAndTxt("phx:Buoyancy:invalidSize", "The point counts must add up to the columns of P.");
    }

    plhs[0] = mxCreateDoubleMatrix(2, T, mxREAL);
    double* XY = mxGetDoubles(plhs[0]);

    mwSize j = 0;
    for (mwSize a = 0; a < n; ++a) {
        const double* m = M + 16*a;
        const mwSize e = j + static_cast<mwSize>(counts[a]);
        for (; j < e; ++j) {
            const double* p = P + 3*j;
            XY[2*j] = m[0]*p[0] + m[4]*p[1] + m[8]*p[2] + m[12];
            XY[2*j + 1] = m[1]*p[0] + m[5]*p[1] + m[9]*p[2] + m[13];
        }
    }
}
