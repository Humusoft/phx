// Copyright 2026 HUMUSOFT s.r.o.
// SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
// Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
//
// MEX build of phx.internal.computeMonopoleForces (the .m file is the fallback).
// Build from phx/+phx/+internal:
//   mex -R2018a -O mex_sources/computeMonopoleForces.cpp
//
#include "mex.h"
#include <cmath>

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    if (nrhs != 3) {
        mexErrMsgIdAndTxt("phx:Monopole:invalidInputs", "Three inputs required.");
    }
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || mxGetM(prhs[0]) != 3) {
        mexErrMsgIdAndTxt("phx:Monopole:invalidSize", "pos must be a real 3xN double array.");
    }
    const mwSize n = mxGetN(prhs[0]);
    if (!mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != n) {
        mexErrMsgIdAndTxt("phx:Monopole:invalidSize", "charge must have one value per body.");
    }
    const double* pos = mxGetDoubles(prhs[0]);
    const double* q = mxGetDoubles(prhs[1]);
    const double k = mxGetScalar(prhs[2]);

    plhs[0] = mxCreateDoubleMatrix(3, n, mxREAL);
    double* F = mxGetDoubles(plhs[0]);

    // Same operation order as the MATLAB loop in phx.Monopole.resolveState
    for (mwSize a = 0; a + 1 < n; ++a) {
        const double ax = pos[3*a], ay = pos[3*a + 1], az = pos[3*a + 2];
        const double qa = q[a];
        double sx = 0.0, sy = 0.0, sz = 0.0;
        for (mwSize b = a + 1; b < n; ++b) {
            const double dx = pos[3*b] - ax;
            const double dy = pos[3*b + 1] - ay;
            const double dz = pos[3*b + 2] - az;
            const double l = std::sqrt(dx*dx + dy*dy + dz*dz);
            const double w = qa*q[b]/(l*l*l);
            const double fx = dx*w, fy = dy*w, fz = dz*w;
            sx += fx; sy += fy; sz += fz;
            F[3*b] -= fx; F[3*b + 1] -= fy; F[3*b + 2] -= fz;
        }
        F[3*a] += sx; F[3*a + 1] += sy; F[3*a + 2] += sz;
    }
    for (mwSize i = 0; i < 3*n; ++i) {
        F[i] *= k;
    }
}
