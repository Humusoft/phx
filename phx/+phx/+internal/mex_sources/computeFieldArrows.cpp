#include "mex.h"
#include <cmath>
#include <cstring>

// Helper to get pointer to column c of a 3xK matrix (column-major)
inline const double* col3(const double* A, mwIndex c) {
    return A + 3*c;
}
inline double* col3(double* A, mwIndex c) {
    return A + 3*c;
}

void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[])
{
    if (nrhs != 6) {
        mexErrMsgIdAndTxt("computeArrows:invalidNumInputs", "Six inputs required.");
    }
    // Input pointers
    const double* grid = mxGetPr(prhs[0]);   // 3 x N
    const double* pos  = mxGetPr(prhs[1]);   // 3 x M
    const double* charge = mxGetPr(prhs[2]); // M x 1
    double count_d = mxGetScalar(prhs[3]);
    double seg_d   = mxGetScalar(prhs[4]);
    double segLen  = mxGetScalar(prhs[5]);

    mwSize count = static_cast<mwSize>(count_d);
    mwSize seg   = static_cast<mwSize>(seg_d);

    if (mxGetM(prhs[0]) != 3 || mxGetM(prhs[1]) != 3) {
        mexErrMsgIdAndTxt("computeArrows:invalidSize", "grid and pos must be 3xK arrays.");
    }

    mwSize Ngrid = mxGetN(prhs[0]);
    mwSize Mpos  = mxGetN(prhs[1]);

    if (count > Ngrid) {
        mexErrMsgIdAndTxt("computeArrows:countTooLarge", "count cannot exceed columns in grid.");
    }
    if (mxGetNumberOfElements(prhs[2]) != Mpos) {
        mexErrMsgIdAndTxt("computeArrows:chargeSize", "charge must have length equal to number of columns in pos.");
    }
    if (seg < 3) {
        mexErrMsgIdAndTxt("computeArrows:segTooSmall", "seg must be >= 3.");
    }

    // Output size: 3 x (count * seg)
    mwSize outCols = count * seg;
    mwSize outDims[2] = {3, outCols};
    plhs[0] = mxCreateNumericArray(2, outDims, mxDOUBLE_CLASS, mxREAL);
    double* XYZ = mxGetPr(plhs[0]);

    // Index pointer (zero-based column index)
    mwSize si = 0;

    // Temporary variables
    double point[3];
    double F[3];
    double arr[3];

    for (mwSize h = 0; h < count; ++h) {
        // point = grid(:, h)
        const double* gcol = col3(grid, h);
        point[0] = gcol[0];
        point[1] = gcol[1];
        point[2] = gcol[2];

        // XYZ(:, si) = point
        double* outcol = col3(XYZ, si);
        outcol[0] = point[0];
        outcol[1] = point[1];
        outcol[2] = point[2];

        // inner loop m = 1:(seg-2)  => executes seg-2 times
        for (mwSize m = 0; m < (seg - 2); ++m) {
            // compute F = sum_j ( (pos(:,j) - point) * charge[j] / r^3 )
            F[0] = F[1] = F[2] = 0.0;
            for (mwSize j = 0; j < Mpos; ++j) {
                const double* pcol = col3(pos, j);
                double dx = pcol[0] - point[0];
                double dy = pcol[1] - point[1];
                double dz = pcol[2] - point[2];
                double r2 = dx*dx + dy*dy + dz*dz;
                double r = std::sqrt(r2);
                double denom = r * r * r; // r^3
                if (denom == 0.0) {
                    continue; // skip singular contribution
                }
                double w = charge[j] / denom;
                F[0] += dx * w;
                F[1] += dy * w;
                F[2] += dz * w;
            }

            // Normalize and scale: F = segLen * F / norm(F)
            double Flen = std::sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2]);
            if (Flen == 0.0) {
                // if zero, no motion; keep F zero
                F[0] = F[1] = F[2] = 0.0;
            } else {
                double s = segLen / Flen;
                F[0] *= s;
                F[1] *= s;
                F[2] *= s;
            }

            // point = point + F
            point[0] += F[0];
            point[1] += F[1];
            point[2] += F[2];

            // increment si and store point
            ++si;
            if (si >= outCols) {
                mexErrMsgIdAndTxt("computeArrows:outOfBounds", "Output index exceeded expected size.");
            }
            double* outcol2 = col3(XYZ, si);
            outcol2[0] = point[0];
            outcol2[1] = point[1];
            outcol2[2] = point[2];
        }

        // After inner loop: compute arr = [-F(2); F(1); F(3)] (note indexing 1-based in MATLAB)
        // Our F = [F0, F1, F2] corresponds to MATLAB F(1),F(2),F(3)
        arr[0] = -F[1];
        arr[1] =  F[0];
        arr[2] =  F[2];

        // XYZ(:, si + 1) = XYZ(:, si - 1) + 0.25*arr;
        // compute target column index = si + 1 (zero-based)
        mwSize target = si + 1;
        if (si < 1 || target >= outCols) {
            mexErrMsgIdAndTxt("computeArrows:outOfBounds2", "Invalid indexing when writing arrow end.");
        }
        double* srccol = col3(XYZ, si - 1);
        double* tgtcol = col3(XYZ, target);
        tgtcol[0] = srccol[0] + 0.25 * arr[0];
        tgtcol[1] = srccol[1] + 0.25 * arr[1];
        tgtcol[2] = srccol[2] + 0.25 * arr[2];

        // increment si by 2 (to match MATLAB's si = si + 2)
        si = target + 1;
    }

    // Done. plhs[0] already set.
}
