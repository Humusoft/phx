// Copyright 2026 HUMUSOFT s.r.o.
// SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
// Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
//
// MEX build of phx.internal.computeDipoleForces (the .m file is the fallback).
// Build from phx/+phx/+internal:
//   mex -R2018a -O mex_sources/computeDipoleForces.cpp
//
#include "mex.h"
#include <cmath>

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    if (nrhs != 3) {
        mexErrMsgIdAndTxt("phx:Dipole:invalidInputs", "Three inputs required.");
    }
    if (!mxIsDouble(prhs[1]) || mxIsComplex(prhs[1])) {
        mexErrMsgIdAndTxt("phx:Dipole:invalidSize", "charge must be a real double vector.");
    }
    const mwSize n = mxGetNumberOfElements(prhs[1]);
    const mwSize m = 2*n;
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || mxGetM(prhs[0]) != 3 || mxGetN(prhs[0]) != m) {
        mexErrMsgIdAndTxt("phx:Dipole:invalidSize", "pos must be a real 3x2N double array, two poles per charge.");
    }
    const double* pos = mxGetDoubles(prhs[0]);
    const double* c = mxGetDoubles(prhs[1]);
    const double k = mxGetScalar(prhs[2]);

    plhs[0] = mxCreateDoubleMatrix(3, m, mxREAL);
    double* F = mxGetDoubles(plhs[0]);

    // Same operation order as the .m fallback; pole a and a + n belong to one body
    for (mwSize a = 0; a + 1 < m; ++a) {
        const double ax = pos[3*a], ay = pos[3*a + 1], az = pos[3*a + 2];
        const double qa = a < n ? c[a] : -c[a - n];
        const mwSize self = a < n ? a + n : m;
        double sx = 0.0, sy = 0.0, sz = 0.0;
        for (mwSize b = a + 1; b < m; ++b) {
            if (b == self) {
                continue;
            }
            const double qb = b < n ? c[b] : -c[b - n];
            const double dx = pos[3*b] - ax;
            const double dy = pos[3*b + 1] - ay;
            const double dz = pos[3*b + 2] - az;
            const double l = std::sqrt(dx*dx + dy*dy + dz*dz);
            const double w = qa*qb/(l*l*l);
            const double fx = dx*w, fy = dy*w, fz = dz*w;
            sx += fx; sy += fy; sz += fz;
            F[3*b] -= fx; F[3*b + 1] -= fy; F[3*b + 2] -= fz;
        }
        F[3*a] += sx; F[3*a + 1] += sy; F[3*a + 2] += sz;
    }
    for (mwSize i = 0; i < 3*m; ++i) {
        F[i] *= k;
    }
}
