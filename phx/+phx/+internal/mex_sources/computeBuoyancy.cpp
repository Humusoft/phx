// Copyright 2026 HUMUSOFT s.r.o.
// SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
// Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
//
// MEX build of phx.internal.computeBuoyancy (the .m file is the fallback).
// Build from phx/+phx/+internal:
//   mex -R2018a -O mex_sources/computeBuoyancy.cpp
//
#include "mex.h"

static bool isRealDouble(const mxArray* a)
{
    return mxIsDouble(a) && !mxIsComplex(a) && !mxIsSparse(a);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    if (nrhs != 5) {
        mexErrMsgIdAndTxt("phx:Buoyancy:invalidInputs", "Five inputs required.");
    }
    for (int i = 0; i < 5; ++i) {
        if (!isRealDouble(prhs[i])) {
            mexErrMsgIdAndTxt("phx:Buoyancy:invalidInputs", "All inputs must be real double arrays.");
        }
    }
    const mwSize n = mxGetNumberOfElements(prhs[3]);
    const mwSize T = mxGetN(prhs[1]);
    if (mxGetM(prhs[0]) != 16 || mxGetN(prhs[0]) != n || mxGetM(prhs[1]) != 3
            || mxGetNumberOfElements(prhs[2]) != T) {
        mexErrMsgIdAndTxt("phx:Buoyancy:invalidSize", "M must be 16xN, P 3xT and dV of T elements for N counts.");
    }
    const mwSize nLevel = mxGetNumberOfElements(prhs[4]);
    if (nLevel != 1 && nLevel != T) {
        mexErrMsgIdAndTxt("phx:Buoyancy:invalidLevel", "The level must be a scalar or have one value per sampling point.");
    }
    const double* M = mxGetDoubles(prhs[0]);
    const double* P = mxGetDoubles(prhs[1]);
    const double* dV = mxGetDoubles(prhs[2]);
    const double* counts = mxGetDoubles(prhs[3]);
    const double* level = mxGetDoubles(prhs[4]);
    const bool flat = nLevel == 1;

    mwSize total = 0;
    for (mwSize a = 0; a < n; ++a) {
        total += static_cast<mwSize>(counts[a]);
    }
    if (total != T) {
        mexErrMsgIdAndTxt("phx:Buoyancy:invalidSize", "The point counts must add up to the columns of P.");
    }

    plhs[0] = mxCreateDoubleMatrix(1, n, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, n, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(3, n, mxREAL);
    double* Vsub = mxGetDoubles(plhs[0]);
    double* Vtot = mxGetDoubles(plhs[1]);
    double* C = mxGetDoubles(plhs[2]);

    mwSize j = 0;
    for (mwSize a = 0; a < n; ++a) {
        const double* m = M + 16*a;
        const mwSize e = j + static_cast<mwSize>(counts[a]);
        double vs = 0.0, vt = 0.0, cx = 0.0, cy = 0.0, cz = 0.0;
        for (; j < e; ++j) {
            const double* p = P + 3*j;
            const double z = m[2]*p[0] + m[6]*p[1] + m[10]*p[2] + m[14];
            const double v = dV[j];
            vt += v;
            if (z < (flat ? level[0] : level[j])) {
                vs += v;
                cx += p[0]*v;
                cy += p[1]*v;
                cz += p[2]*v;
            }
        }
        Vsub[a] = vs;
        Vtot[a] = vt;
        if (vs > 0.0) {
            C[3*a] = cx/vs;
            C[3*a + 1] = cy/vs;
            C[3*a + 2] = cz/vs;
        }
    }
}
