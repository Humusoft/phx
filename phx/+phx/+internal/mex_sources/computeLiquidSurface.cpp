// Copyright 2026 HUMUSOFT s.r.o.
// SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
// Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
//
// MEX build of phx.internal.computeLiquidSurface (the .m file is the fallback).
// Build from phx/+phx/+internal:
//   mex -R2018a -O mex_sources/computeLiquidSurface.cpp
//
// [V, F, N] = computeLiquidSurface(P, d): the surface of a liquid of
// particles at P (3xN) of rest spacing d, as vertices V (3xK), triangles F
// (Mx3, one-based) and unit vertex normals N (3xK) pointing out of the
// liquid. The field is that of the .m file: the particles spread
// trilinearly onto a grid of cells d/2, smoothed by a separable Gaussian of
// 0.6 d, normalised so that a liquid at rest reads 1; the surface is its
// level 0.5, one grid per cluster of touching blocks of 4 d. The .m file
// triangulates the level with isosurface; here every cube that the level
// crosses gets the polygons its faces trace (each face joins its crossings
// in pairs, a face with four of them by the value at its centre, which the
// cube beside reads alike), fanned into triangles. The vertices on the grid
// edges are those of the .m file, the triangles may differ where a cube is
// ambiguous; the surface is closed either way.
//
#include "mex.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {

const double Iso = 0.5;

// Corners of a cube: bit 0 = +x, bit 1 = +y, bit 2 = +z. The faces with
// their corners counter-clockwise seen from outside.
const int FaceCorners[6][4] = {{0, 4, 6, 2}, {1, 3, 7, 5}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 2, 3, 1}, {4, 5, 7, 6}};

struct Edges {
    int id[8][8];       // local edge of a corner pair, -1 if not an edge
    int low[12], axis[12];
    Edges()
    {
        for (int a = 0; a < 8; ++a)
            for (int b = 0; b < 8; ++b)
                id[a][b] = -1;
        int n = 0;
        for (int c = 0; c < 8; ++c)
            for (int ax = 0; ax < 3; ++ax)
                if (!(c & (1 << ax))) {
                    int d = c | (1 << ax);
                    id[c][d] = id[d][c] = n;
                    low[n] = c;
                    axis[n] = ax;
                    ++n;
                }
    }
};
const Edges CubeEdges;

// The surface of one cluster of particles, appended to V, F
void surfaceOf(const double *P, const std::vector<std::uint32_t> &members, double d,
    std::vector<double> &V, std::vector<std::uint32_t> &F)
{
    const double h = d/2;
    const double sigma = 0.6*d/h;
    const int kr = int(std::ceil(2.5*sigma));
    const double pad = (kr + 2)*h;

    double lo[3] = {HUGE_VAL, HUGE_VAL, HUGE_VAL}, hi[3] = {-HUGE_VAL, -HUGE_VAL, -HUGE_VAL};
    for (std::uint32_t i : members)
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], P[3*i + a]);
            hi[a] = std::max(hi[a], P[3*i + a]);
        }
    std::int64_t dims[3];
    for (int a = 0; a < 3; ++a) {
        lo[a] -= pad;
        dims[a] = std::int64_t(std::ceil((hi[a] + pad - lo[a])/h)) + 1;
    }
    const std::int64_t sx = 1, sy = dims[0], sz = dims[0]*dims[1], nNodes = dims[0]*dims[1]*dims[2];

    // Trilinear splat
    std::vector<double> G(size_t(nNodes), 0.0);
    for (std::uint32_t i : members) {
        std::int64_t i0[3];
        double f[3];
        for (int a = 0; a < 3; ++a) {
            double u = (P[3*i + a] - lo[a])/h;
            double fl = std::floor(u);
            i0[a] = std::int64_t(fl);
            f[a] = u - fl;
        }
        for (int dz = 0; dz < 2; ++dz)
            for (int dy = 0; dy < 2; ++dy)
                for (int dx = 0; dx < 2; ++dx) {
                    double w = (dx ? f[0] : 1 - f[0])*(dy ? f[1] : 1 - f[1])*(dz ? f[2] : 1 - f[2]);
                    G[size_t((i0[0] + dx)*sx + (i0[1] + dy)*sy + (i0[2] + dz)*sz)] += w;
                }
    }

    // Separable Gaussian along each axis ("same", zeros outside), then the
    // normalisation (d/h)^3
    std::vector<double> k(size_t(2*kr + 1));
    double ksum = 0;
    for (int j = -kr; j <= kr; ++j)
        ksum += (k[size_t(j + kr)] = std::exp(-0.5*(j/sigma)*(j/sigma)));
    for (double &kj : k)
        kj /= ksum;
    std::vector<double> T(static_cast<size_t>(nNodes));
    const std::int64_t strides[3] = {sx, sy, sz};
    // Along x the lines are contiguous; along y and z whole rows along x are
    // added at once, so that the innermost loop runs through memory
    for (std::int64_t line = 0; line < nNodes; line += dims[0])
        for (std::int64_t pos = 0; pos < dims[0]; ++pos) {
            double acc = 0;
            const std::int64_t j0 = std::max<std::int64_t>(-kr, -pos), j1 = std::min<std::int64_t>(kr, dims[0] - 1 - pos);
            for (std::int64_t j = j0; j <= j1; ++j)
                acc += k[size_t(j + kr)]*G[size_t(line + pos + j)];
            T[size_t(line + pos)] = acc;
        }
    G.swap(T);
    for (int a = 1; a < 3; ++a) {
        const int other = 3 - a;   // the axis besides x and a
        const std::int64_t s = strides[a], n = dims[a];
        std::fill(T.begin(), T.end(), 0.0);
        for (std::int64_t io = 0; io < dims[other]; ++io)
            for (std::int64_t pos = 0; pos < n; ++pos) {
                double *dst = &T[size_t(io*strides[other] + pos*s)];
                const std::int64_t j0 = std::max<std::int64_t>(-kr, -pos), j1 = std::min<std::int64_t>(kr, n - 1 - pos);
                for (std::int64_t j = j0; j <= j1; ++j) {
                    const double kj = k[size_t(j + kr)];
                    const double *src = &G[size_t(io*strides[other] + (pos + j)*s)];
                    for (std::int64_t x = 0; x < dims[0]; ++x)
                        dst[x] += kj*src[x];
                }
            }
        G.swap(T);
    }
    const double scale = (d/h)*(d/h)*(d/h);
    for (double &g : G)
        g *= scale;

    // Polygons of the crossed cubes; one vertex per crossed grid edge
    std::vector<std::int32_t> edgeVertex(size_t(3*nNodes), -1);
    const std::int64_t cornerOffset[8] = {0, sx, sy, sx + sy, sz, sx + sz, sy + sz, sx + sy + sz};
    for (std::int64_t iz = 0; iz + 1 < dims[2]; ++iz)
        for (std::int64_t iy = 0; iy + 1 < dims[1]; ++iy)
            for (std::int64_t ix = 0; ix + 1 < dims[0]; ++ix) {
                const std::int64_t base = ix*sx + iy*sy + iz*sz;
                double v[8];
                bool in[8];
                int mask = 0;
                for (int c = 0; c < 8; ++c) {
                    v[c] = G[size_t(base + cornerOffset[c])];
                    in[c] = v[c] > Iso;
                    mask |= int(in[c]) << c;
                }
                if (mask == 0 || mask == 255)
                    continue;

                std::int32_t vert[12];
                int next[12];
                for (int e = 0; e < 12; ++e) {
                    vert[e] = -1;
                    next[e] = -1;
                }
                auto vertexOf = [&](int e) {
                    if (vert[e] >= 0)
                        return vert[e];
                    const int c0 = CubeEdges.low[e], ax = CubeEdges.axis[e];
                    const size_t key = size_t(3*(base + cornerOffset[c0]) + ax);
                    if (edgeVertex[key] < 0) {
                        const int c1 = c0 | (1 << ax);
                        const double t = (Iso - v[c0])/(v[c1] - v[c0]);
                        const double node[3] = {double(ix + (c0 & 1)), double(iy + ((c0 >> 1) & 1)), double(iz + ((c0 >> 2) & 1))};
                        edgeVertex[key] = std::int32_t(V.size()/3);
                        for (int a = 0; a < 3; ++a)
                            V.push_back(lo[a] + (node[a] + (a == ax ? t : 0.0))*h);
                    }
                    return vert[e] = edgeVertex[key];
                };

                // Each face joins its crossings in pairs, from where the
                // level enters the inside (counter-clockwise) to where it
                // leaves; four crossings are paired around the inside
                // corners or across the inside centre
                for (int f = 0; f < 6; ++f) {
                    int ce[4];
                    bool entry[4];
                    int nc = 0;
                    for (int s = 0; s < 4; ++s) {
                        const int a = FaceCorners[f][s], b = FaceCorners[f][(s + 1) % 4];
                        if (in[a] != in[b]) {
                            ce[nc] = CubeEdges.id[a][b];
                            entry[nc] = in[b];
                            ++nc;
                        }
                    }
                    if (nc == 2) {
                        next[entry[0] ? ce[0] : ce[1]] = entry[0] ? ce[1] : ce[0];
                    } else if (nc == 4) {
                        double centre = 0;
                        for (int s = 0; s < 4; ++s)
                            centre += v[FaceCorners[f][s]];
                        const bool joined = centre/4 > Iso;
                        for (int q = 0; q < 4; ++q)
                            if (entry[q])
                                next[ce[q]] = ce[joined ? (q + 3) % 4 : (q + 1) % 4];
                    }
                }

                // The loops of crossings, fanned into triangles
                bool done[12] = {false};
                for (int e = 0; e < 12; ++e) {
                    if (next[e] < 0 || done[e])
                        continue;
                    std::int32_t loop[12];
                    int m = 0;
                    int cur = e;
                    while (cur >= 0 && !done[cur] && m < 12) {
                        done[cur] = true;
                        loop[m++] = vertexOf(cur);
                        cur = next[cur];
                    }
                    for (int q = 1; q + 1 < m; ++q) {
                        F.push_back(std::uint32_t(loop[0]));
                        F.push_back(std::uint32_t(loop[q]));
                        F.push_back(std::uint32_t(loop[q + 1]));
                    }
                }
            }
}

} // namespace

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 2)
        mexErrMsgIdAndTxt("phx:Liquid:invalidInputs", "Two inputs required: P and d.");
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || (mxGetM(prhs[0]) != 3 && !mxIsEmpty(prhs[0])))
        mexErrMsgIdAndTxt("phx:Liquid:invalidSize", "P must be a real 3xN double array.");
    const size_t n = mxIsEmpty(prhs[0]) ? 0 : mxGetN(prhs[0]);
    const double *P = mxGetDoubles(prhs[0]);
    const double d = mxGetScalar(prhs[1]);
    if (!(d > 0))
        mexErrMsgIdAndTxt("phx:Liquid:invalidSpacing", "d must be positive.");

    std::vector<double> V;
    std::vector<std::uint32_t> F;
    if (n > 0) {
        // Blocks of 4 d; touching ones (by a face, an edge or a corner) form
        // a cluster, with a grid of its own. A particle's field reaches 2 d
        // (one cell of the splat, three of the Gaussian), so the fields of
        // blocks that do not touch never meet.
        double lo[3] = {HUGE_VAL, HUGE_VAL, HUGE_VAL};
        for (size_t i = 0; i < n; ++i)
            for (int a = 0; a < 3; ++a)
                lo[a] = std::min(lo[a], P[3*i + a]);
        const double b = 4*d;
        const std::int64_t Bits = 21, Max = (std::int64_t(1) << Bits) - 1;
        auto pack = [&](std::int64_t x, std::int64_t y, std::int64_t z) {
            return std::uint64_t(z) << (2*Bits) | std::uint64_t(y) << Bits | std::uint64_t(x);
        };
        std::vector<std::uint64_t> blockOf(n);
        for (size_t i = 0; i < n; ++i) {
            std::int64_t c[3];
            for (int a = 0; a < 3; ++a)
                c[a] = std::min<std::int64_t>(Max - 1, std::int64_t(std::floor((P[3*i + a] - lo[a])/b)) + 1);
            blockOf[i] = pack(c[0], c[1], c[2]);
        }
        std::vector<std::uint64_t> blocks(blockOf);
        std::sort(blocks.begin(), blocks.end());
        blocks.erase(std::unique(blocks.begin(), blocks.end()), blocks.end());
        std::vector<size_t> parent(blocks.size());
        for (size_t q = 0; q < parent.size(); ++q)
            parent[q] = q;
        auto root = [&](size_t q) {
            while (parent[q] != q)
                q = parent[q] = parent[parent[q]];
            return q;
        };
        for (size_t q = 0; q < blocks.size(); ++q) {
            const std::int64_t x = std::int64_t(blocks[q] & std::uint64_t(Max)), y = std::int64_t((blocks[q] >> Bits) & std::uint64_t(Max)),
                z = std::int64_t(blocks[q] >> (2*Bits));
            for (int dz = -1; dz <= 1; ++dz)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const std::uint64_t key = pack(x + dx, y + dy, z + dz);
                        auto it = std::lower_bound(blocks.begin(), blocks.end(), key);
                        if (it != blocks.end() && *it == key) {
                            size_t r1 = root(q), r2 = root(size_t(it - blocks.begin()));
                            if (r1 != r2)
                                parent[std::max(r1, r2)] = std::min(r1, r2);
                        }
                    }
        }
        std::vector<std::vector<std::uint32_t>> clusters(blocks.size());
        for (size_t i = 0; i < n; ++i) {
            size_t q = size_t(std::lower_bound(blocks.begin(), blocks.end(), blockOf[i]) - blocks.begin());
            clusters[root(q)].push_back(std::uint32_t(i));
        }
        for (const auto &members : clusters)
            if (!members.empty())
                surfaceOf(P, members, d, V, F);
    }

    // Outputs; the normals are the area-weighted sums of the face normals
    const size_t nv = V.size()/3, nf = F.size()/3;
    plhs[0] = mxCreateDoubleMatrix(3, nv, mxREAL);
    std::copy(V.begin(), V.end(), mxGetDoubles(plhs[0]));
    if (nlhs > 1) {
        plhs[1] = mxCreateDoubleMatrix(nf, 3, mxREAL);
        double *Fo = mxGetDoubles(plhs[1]);
        for (size_t t = 0; t < nf; ++t)
            for (int c = 0; c < 3; ++c)
                Fo[t + c*nf] = double(F[3*t + c]) + 1;
    }
    if (nlhs > 2) {
        plhs[2] = mxCreateDoubleMatrix(3, nv, mxREAL);
        double *N = mxGetDoubles(plhs[2]);
        for (size_t t = 0; t < nf; ++t) {
            const double *a = &V[3*F[3*t]], *b = &V[3*F[3*t + 1]], *c = &V[3*F[3*t + 2]];
            const double e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
            const double e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
            const double fn[3] = {e1[1]*e2[2] - e1[2]*e2[1], e1[2]*e2[0] - e1[0]*e2[2], e1[0]*e2[1] - e1[1]*e2[0]};
            for (int corner = 0; corner < 3; ++corner)
                for (int a2 = 0; a2 < 3; ++a2)
                    N[3*F[3*t + corner] + a2] += fn[a2];
        }
        for (size_t q = 0; q < nv; ++q) {
            double len = std::sqrt(N[3*q]*N[3*q] + N[3*q + 1]*N[3*q + 1] + N[3*q + 2]*N[3*q + 2]);
            len = std::max(len, 2.2250738585072014e-308);
            for (int a2 = 0; a2 < 3; ++a2)
                N[3*q + a2] /= len;
        }
    }
}
