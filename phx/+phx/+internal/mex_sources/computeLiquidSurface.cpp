// Copyright 2026 HUMUSOFT s.r.o.
// SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
// Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
//
// MEX build of phx.internal.computeLiquidSurface (the .m file is the fallback).
// Build from phx/+phx/+internal:
//   mex -R2018a -O mex_sources/computeLiquidSurface.cpp
//
// [V, F, N] = computeLiquidSurface(P, d [, threads]): the surface of a
// liquid of particles at P (3xN) of rest spacing d, as vertices V (3xK
// single), triangles F (3xM uint32, one-based, a column per triangle) and
// unit vertex normals N (3xK single) pointing out of the liquid: the types
// a TriangleStrip takes, F(:)' its VertexIndices. The field is that of the
// .m file: the particles spread trilinearly onto the nodes of a grid of
// cells d/2 aligned with the world origin, smoothed by a separable Gaussian
// of 0.6 d, normalised so that a liquid at rest reads 1; the surface is its
// level 0.5. The field is single precision: the vertices lie within about
// 1e-5 cells of those of the .m file.
//
// The grid is sparse: bricks of 8x8x8 nodes, only those the field of some
// particle reaches, so a drop far from the rest of the liquid costs a few
// bricks instead of a grid stretched to it, and empty space costs nothing.
// The .m file triangulates the level with isosurface; here every cube that
// the level crosses gets the polygons its faces trace (each face joins its
// crossings in pairs, a face with four of them by the value at its centre,
// which the cube beside reads alike), fanned into triangles, and bricks the
// level does not cross are skipped whole. The vertices on the grid edges
// are those of the .m file, the triangles may differ where a cube is
// ambiguous; the surface is closed either way.
//
// The work runs brick by brick on up to `threads` threads (default
// maxNumCompThreads), the calling one among them, fewer where a pass has
// little work, with the same result on any number. The helper threads are
// kept between calls and wait asleep (a condition variable, no spinning),
// so they take no time from the physics engine or MATLAB between calls;
// they end when the MEX file is cleared. They never call MATLAB. A pass
// ends when its work is done, without waiting for a helper that has not
// started (on a busy computer one may not get a core in time).
//
// The Gaussian, the range of each brick and the inside bits of the cubes
// use AVX2 where the processor and the system have it, checked at run time;
// the file is built for the x64 baseline, so it loads on any processor, and
// the scalar code runs elsewhere. Both give the same results to the bit.
// The grid and the vertex table are kept between calls, for the time their
// allocation takes, until the MEX file is cleared.
//
#include "mex.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <memory>
#include <thread>
#include <vector>

// AVX2 on x64 where the CPU and the OS have it, chosen at run time; the
// file itself is built for the baseline (SSE2), so it loads anywhere
#if defined(_M_X64) || defined(__x86_64__)
#define PHX_SURFACE_AVX2 1
#include <immintrin.h>
#if defined(_MSC_VER)
#include <intrin.h>
#define PHX_AVX2_FN
#else
#include <cpuid.h>
#define PHX_AVX2_FN __attribute__((target("avx2")))
#endif
#endif

namespace {

#ifdef PHX_SURFACE_AVX2
// CPUID.7.0:EBX bit 5 (AVX2), and the OS saving the YMM state (OSXSAVE and
// XCR0 bits 1 and 2)
bool detectAVX2()
{
#if defined(_MSC_VER)
    int r[4];
    __cpuid(r, 0);
    if (r[0] < 7)
        return false;
    __cpuid(r, 1);
    const unsigned features = unsigned(r[2]);
    __cpuidex(r, 7, 0);
    const unsigned extended = unsigned(r[1]);
#else
    unsigned a, b, c, d;
    if (__get_cpuid_max(0, nullptr) < 7)
        return false;
    __cpuid(1, a, b, c, d);
    const unsigned features = c;
    __cpuid_count(7, 0, a, b, c, d);
    const unsigned extended = b;
#endif
    if (!(features & (1u << 27)) || !(extended & (1u << 5)))
        return false;
#if defined(_MSC_VER)
    const unsigned long long xcr0 = _xgetbv(0);
#else
    unsigned lo, hi;
    __asm__ volatile("xgetbv" : "=a"(lo), "=d"(hi) : "c"(0));
    const unsigned long long xcr0 = (static_cast<unsigned long long>(hi) << 32) | lo;
#endif
    return (xcr0 & 6) == 6;
}
const bool HasAVX2 = detectAVX2();
#else
const bool HasAVX2 = false;
#endif

// Work arrays kept between calls, uninitialised, grown with some headroom
template <class T> class Buffer {
public:
    T *get(size_t n)
    {
        if (n > capacity_) {
            data_.reset();
            capacity_ = n + n/4;
            data_.reset(new T[capacity_]);
        }
        return data_.get();
    }
private:
    std::unique_ptr<T[]> data_;
    size_t capacity_ = 0;
};
Buffer<float> fieldBuffer, blurBuffer;
Buffer<std::int32_t> tableBuffer;

const float Iso = 0.5f;

// The Gaussian in cells of d/2: sigma 0.6 d, cut at 2.5 sigma
const double Sigma = 1.2;
const int KR = 3;

// Nodes along each edge of a brick, and in a brick
const int LogB = 3;
const int B = 1 << LogB;
const int B3 = B*B*B;
// Nodes of a brick with those of the bricks above it, for its cubes
const int C = B + 1;

// Index of a node in its brick, from coordinates that may reach B (those
// of the next brick, which the caller has picked)
inline int local(int x, int y, int z)
{
    return (x & (B - 1)) + B*(y & (B - 1)) + B*B*(z & (B - 1));
}

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

// Brick coordinates packed into a key, 21 bits each, offset by one so that
// the neighbours of brick 0 have keys too
const int Bits = 21;
const std::int64_t MaxCoord = (std::int64_t(1) << Bits) - 2;

std::uint64_t pack(std::int64_t x, std::int64_t y, std::int64_t z)
{
    return std::uint64_t(z + 1) << (2*Bits) | std::uint64_t(y + 1) << Bits | std::uint64_t(x + 1);
}

// Open-addressing map from a key to an index, in the order of insertion
class KeyMap {
public:
    explicit KeyMap(size_t expected)
    {
        size_t capacity = 16;
        while (capacity < 2*expected)
            capacity <<= 1;
        // (a copy of Empty: bound to the reference itself it would need a
        // definition outside the class before C++17, as mex uses on Linux)
        keys_.assign(capacity, std::uint64_t(Empty));
        values_.assign(capacity, -1);
        mask_ = capacity - 1;
    }

    std::int32_t find(std::uint64_t key) const
    {
        for (size_t s = slot(key);; s = (s + 1) & mask_) {
            if (keys_[s] == key)
                return values_[s];
            if (keys_[s] == Empty)
                return -1;
        }
    }

    // The index of the key, value if it is new
    std::int32_t insert(std::uint64_t key, std::int32_t value)
    {
        for (size_t s = slot(key);; s = (s + 1) & mask_) {
            if (keys_[s] == key)
                return values_[s];
            if (keys_[s] == Empty) {
                keys_[s] = key;
                values_[s] = value;
                return value;
            }
        }
    }

private:
    static const std::uint64_t Empty = ~std::uint64_t(0);
    std::vector<std::uint64_t> keys_;
    std::vector<std::int32_t> values_;
    size_t mask_ = 0;

    size_t slot(std::uint64_t k) const
    {
        k ^= k >> 33;
        k *= 0xff51afd7ed558ccdULL;
        k ^= k >> 33;
        return size_t(k) & mask_;
    }
};

// Index of a neighbour among the 27 of a brick, offsets -1..1 per axis
int neighbour(int dx, int dy, int dz)
{
    return (dx + 1) + 3*(dy + 1) + 9*(dz + 1);
}

// Helper threads kept between calls. run(count, threads, f, grain) calls
// f(i) for i = 0..count-1, on the calling thread and up to threads - 1
// helpers, at most one helper per grain items beyond the first grain; each
// takes the next i as it finishes one. It returns when all items are done:
// a helper that has not started by then (no core for it yet) is not waited
// for, it finds the job closed when it wakes. Idle helpers wait on a
// condition variable.
class Pool {
public:
    ~Pool()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        wake_.notify_all();
        for (std::thread &t : workers_)
            t.join();
    }

    void run(size_t count, unsigned threads, const std::function<void(size_t)> &f, size_t grain = 1)
    {
        grain = std::max<size_t>(grain, 1);
        const size_t byWork = (count + grain - 1)/grain;
        const size_t helpers = std::min<size_t>(threads > 1 ? threads - 1 : 0, byWork > 1 ? byWork - 1 : 0);
        if (helpers == 0) {
            for (size_t i = 0; i < count; ++i)
                f(i);
            return;
        }
        while (workers_.size() < helpers) {
            const unsigned id = unsigned(workers_.size());
            const std::uint64_t now = generation_;
            workers_.emplace_back([this, id, now] { loop(id, now); });
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            job_ = &f;
            count_ = count;
            next_.store(0);
            done_.store(0);
            wanted_ = unsigned(helpers);
            ++generation_;
            open_.store(generation_);
        }
        wake_.notify_all();
        work();
        std::unique_lock<std::mutex> lock(mutex_);
        finished_.wait(lock, [this] { return done_.load() == count_; });
        // Close the job; wait only for the helpers inside it, which have
        // nothing left to take and are leaving
        open_.store(0);
        wanted_ = 0;
        finished_.wait(lock, [this] { return active_.load() == 0; });
        job_ = nullptr;
    }

private:
    void work()
    {
        for (size_t i = next_.fetch_add(1); i < count_; i = next_.fetch_add(1)) {
            (*job_)(i);
            if (done_.fetch_add(1) + 1 == count_) {
                std::lock_guard<std::mutex> lock(mutex_);
                finished_.notify_all();
            }
        }
    }

    void loop(unsigned id, std::uint64_t seen)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        for (;;) {
            wake_.wait(lock, [&] { return stopping_ || (generation_ != seen && id < wanted_); });
            if (stopping_)
                return;
            seen = generation_;
            lock.unlock();
            // Enter (sequentially consistent with the caller's close: either
            // the job is seen open, or the caller sees this helper active)
            active_.fetch_add(1);
            if (open_.load() == seen)
                work();
            lock.lock();
            if (active_.fetch_sub(1) == 1)
                finished_.notify_all();
        }
    }

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable wake_, finished_;
    const std::function<void(size_t)> *job_ = nullptr;
    size_t count_ = 0;
    std::atomic<size_t> next_{0}, done_{0};
    std::atomic<unsigned> active_{0};
    std::atomic<std::uint64_t> open_{0};
    unsigned wanted_ = 0;
    std::uint64_t generation_ = 0;
    bool stopping_ = false;
};

Pool *pool = nullptr;


void endPool()
{
    delete pool;
    pool = nullptr;
}

#ifdef PHX_SURFACE_AVX2
PHX_AVX2_FN void insideAVX2(const float *cube, unsigned *inside);
#endif

// The nodes of brick q and of the bricks above it (C^3, a missing brick
// reads 0), and which of them lie inside, a bit per node along x
void gather(const float *G, const std::int32_t *around, size_t q, float *cube, unsigned *inside)
{
    const float *g = G + q*B3;
    for (int z = 0; z < B; ++z)
        for (int y = 0; y < B; ++y)
            std::copy(g + B*y + B*B*z, g + B*y + B*B*z + B, cube + C*y + C*C*z);
    for (int z = 0; z < C; ++z)
        for (int y = 0; y < C; ++y)
            for (int x = (y < B && z < B) ? B : 0; x < C; ++x) {
                const std::int32_t r = around[neighbour(x == B, y == B, z == B)];
                cube[x + C*y + C*C*z] = r >= 0 ? G[size_t(r)*B3 + local(x, y, z)] : 0.0f;
            }
#ifdef PHX_SURFACE_AVX2
    if (HasAVX2) {
        insideAVX2(cube, inside);
        return;
    }
#endif
    for (int row = 0; row < C*C; ++row) {
        unsigned bits = 0;
        for (int x = 0; x < C; ++x)
            bits |= unsigned(cube[row*C + x] > Iso) << x;
        inside[row] = bits;
    }
}

// The Gaussian along x of a brick g (rows of B nodes), its neighbours
// below and above along x gb and ga (null if missing), into t
void blurXScalar(const float *g, const float *gb, const float *ga, float *t, const float *k)
{
    float line[B + 2*KR];
    for (int row = 0; row < B*B; ++row) {
        const int base = row*B;
        for (int j = 0; j < KR; ++j) {
            line[j] = gb ? gb[base + B - KR + j] : 0.0f;
            line[KR + B + j] = ga ? ga[base + j] : 0.0f;
        }
        std::copy(g + base, g + base + B, line + KR);
        for (int p = 0; p < B; ++p) {
            float acc = 0;
            for (int j = 0; j <= 2*KR; ++j)
                acc += k[j]*line[p + j];
            t[base + p] = acc;
        }
    }
}

// The Gaussian along y (s = B, so = B*B) or z (s = B*B, so = B), whole rows
// along x at once, times factor; with range, the least and greatest value
// written
void blurYZScalar(const float *g, const float *gb, const float *ga, float *t, const float *k,
    int s, int so, float factor, float *range)
{
    float lo = HUGE_VALF, hi = -HUGE_VALF;
    for (int o = 0; o < B; ++o)
        for (int p = 0; p < B; ++p) {
            float acc[B] = {0};
            for (int j = 0; j <= 2*KR; ++j) {
                const int r = p - KR + j;
                const float *row = r < 0 ? (gb ? gb + (r + B)*s : nullptr) : r < B ? g + r*s : (ga ? ga + (r - B)*s : nullptr);
                if (row == nullptr)
                    continue;
                row += o*so;
                for (int x = 0; x < B; ++x)
                    acc[x] += k[j]*row[x];
            }
            for (int x = 0; x < B; ++x) {
                const float v = factor*acc[x];
                t[o*so + p*s + x] = v;
                lo = std::min(lo, v);
                hi = std::max(hi, v);
            }
        }
    if (range != nullptr) {
        range[0] = lo;
        range[1] = hi;
    }
}

#ifdef PHX_SURFACE_AVX2
// The same in AVX2, a row of B = 8 nodes a register. Each lane takes the
// steps of the scalar code in the same order (a product, then a sum; no
// fused multiply-add), so the results are the same to the bit. Only AVX
// instructions between the loads and the final vzeroupper, none of the
// legacy SSE ones the rest of the file is built with.
static_assert(B == 8, "the AVX2 code takes a brick row as one register");

PHX_AVX2_FN void insideAVX2(const float *cube, unsigned *inside)
{
    const __m256 iso = _mm256_set1_ps(Iso);
    for (int row = 0; row < C*C; ++row) {
        const float *r = cube + row*C;
        const unsigned low = unsigned(_mm256_movemask_ps(_mm256_cmp_ps(_mm256_loadu_ps(r), iso, _CMP_GT_OQ)));
        const unsigned high = unsigned(_mm256_movemask_ps(_mm256_cmp_ps(_mm256_loadu_ps(r + 1), iso, _CMP_GT_OQ)));
        inside[row] = low | (high >> (B - 1)) << B;
    }
    _mm256_zeroupper();
}

PHX_AVX2_FN void blurXAVX2(const float *g, const float *gb, const float *ga, float *t, const float *k)
{
    // Lane p of window j reads node p + j of the line: below[5..7],
    // own[0..7], above[0..2]. Blend the neighbour's lanes into the own row
    // where the window takes them, then rotate by j - KR.
    const __m256 zero = _mm256_setzero_ps();
    __m256 kv[2*KR + 1], mask[2*KR + 1];
    __m256i rot[2*KR + 1];
    for (int j = 0; j <= 2*KR; ++j) {
        kv[j] = _mm256_set1_ps(k[j]);
        int m[8], r[8];
        for (int l = 0; l < 8; ++l) {
            r[l] = (l + j - KR) & (B - 1);
            m[l] = j < KR ? -int(l >= B - KR + j) : -int(l <= j - KR - 1);
        }
        rot[j] = _mm256_setr_epi32(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
        mask[j] = _mm256_castsi256_ps(_mm256_setr_epi32(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7]));
    }
    for (int row = 0; row < B*B; ++row) {
        const int base = row*B;
        const __m256 own = _mm256_loadu_ps(g + base);
        const __m256 below = gb ? _mm256_loadu_ps(gb + base) : zero;
        const __m256 above = ga ? _mm256_loadu_ps(ga + base) : zero;
        __m256 acc = zero;
        for (int j = 0; j <= 2*KR; ++j) {
            const __m256 w = j == KR ? own :
                _mm256_permutevar8x32_ps(_mm256_blendv_ps(own, j < KR ? below : above, mask[j]), rot[j]);
            acc = _mm256_add_ps(acc, _mm256_mul_ps(kv[j], w));
        }
        _mm256_storeu_ps(t + base, acc);
    }
    _mm256_zeroupper();
}

PHX_AVX2_FN void blurYZAVX2(const float *g, const float *gb, const float *ga, float *t, const float *k,
    int s, int so, float factor, float *range)
{
    __m256 kv[2*KR + 1];
    for (int j = 0; j <= 2*KR; ++j)
        kv[j] = _mm256_set1_ps(k[j]);
    const __m256 f = _mm256_set1_ps(factor);
    __m256 lo = _mm256_set1_ps(HUGE_VALF), hi = _mm256_set1_ps(-HUGE_VALF);
    for (int o = 0; o < B; ++o)
        for (int p = 0; p < B; ++p) {
            __m256 acc = _mm256_setzero_ps();
            for (int j = 0; j <= 2*KR; ++j) {
                const int r = p - KR + j;
                const float *row = r < 0 ? (gb ? gb + (r + B)*s : nullptr) : r < B ? g + r*s : (ga ? ga + (r - B)*s : nullptr);
                if (row == nullptr)
                    continue;
                acc = _mm256_add_ps(acc, _mm256_mul_ps(kv[j], _mm256_loadu_ps(row + o*so)));
            }
            const __m256 v = _mm256_mul_ps(f, acc);
            _mm256_storeu_ps(t + o*so + p*s, v);
            lo = _mm256_min_ps(lo, v);
            hi = _mm256_max_ps(hi, v);
        }
    alignas(32) float l8[8], h8[8];
    _mm256_store_ps(l8, lo);
    _mm256_store_ps(h8, hi);
    _mm256_zeroupper();
    if (range != nullptr) {
        range[0] = *std::min_element(l8, l8 + 8);
        range[1] = *std::max_element(h8, h8 + 8);
    }
}
#endif

void blurX(const float *g, const float *gb, const float *ga, float *t, const float *k)
{
#ifdef PHX_SURFACE_AVX2
    if (HasAVX2)
        return blurXAVX2(g, gb, ga, t, k);
#endif
    blurXScalar(g, gb, ga, t, k);
}

void blurYZ(const float *g, const float *gb, const float *ga, float *t, const float *k,
    int s, int so, float factor, float *range)
{
#ifdef PHX_SURFACE_AVX2
    if (HasAVX2)
        return blurYZAVX2(g, gb, ga, t, k, s, so, factor, range);
#endif
    blurYZScalar(g, gb, ga, t, k, s, so, factor, range);
}

// The surface, brick by brick: vertices (3 per vertex) and triangles
// (global vertex indices, from 0) of each crossed brick, in brick order
struct Mesh {
    std::vector<std::vector<float>> vertices;
    std::vector<std::vector<std::uint32_t>> triangles;
    std::vector<size_t> vertexStart, triangleStart;   // per crossed brick, and the totals at the end
    size_t bricks = 0;
};

void surface(const double *P, size_t n, double d, unsigned threads, Mesh &mesh)
{
    const double h = d/2;
    const double pad = (KR + 2)*h;

    // Grid origin on the world lattice of cells h, room for the kernel
    // below the particles: node indices of the field from KR + 2 - 1 up
    double lo[3] = {HUGE_VAL, HUGE_VAL, HUGE_VAL};
    for (size_t i = 0; i < n; ++i)
        for (int a = 0; a < 3; ++a)
            lo[a] = std::min(lo[a], P[3*i + a]);
    for (int a = 0; a < 3; ++a)
        lo[a] = std::floor((lo[a] - pad)/h)*h;

    // The splat node of each particle and its fractions
    std::vector<std::int64_t> node(3*n);
    std::vector<float> frac(3*n);
    for (size_t i = 0; i < n; ++i)
        for (int a = 0; a < 3; ++a) {
            const double u = (P[3*i + a] - lo[a])/h;
            const double fl = std::floor(u);
            node[3*i + a] = std::int64_t(fl);
            frac[3*i + a] = float(u - fl);
            if ((node[3*i + a] >> LogB) > MaxCoord)
                mexErrMsgIdAndTxt("phx:Liquid:tooLarge", "The liquid spreads too far for its spacing.");
        }

    // The bricks of the particles' nodes, their particles in order, and
    // the range of nodes those reach: the splat (one node up), the Gaussian
    // (KR either way), and one node below, so that every cube and grid
    // edge the level can cross starts in a brick
    KeyMap ofParticles(n);
    std::vector<std::int64_t> reach;   // 6 per brick of particles: low and high node per axis
    std::vector<std::uint64_t> particleBrickKey;
    std::vector<std::int32_t> particleBrick(n);
    for (size_t i = 0; i < n; ++i) {
        const std::int64_t *c = &node[3*i];
        const std::uint64_t key = pack(c[0] >> LogB, c[1] >> LogB, c[2] >> LogB);
        const std::int32_t q = ofParticles.insert(key, std::int32_t(particleBrickKey.size()));
        if (size_t(q) == particleBrickKey.size()) {
            particleBrickKey.push_back(key);
            for (int a = 0; a < 3; ++a) {
                reach.push_back(c[a]);
                reach.push_back(c[a]);
            }
        }
        particleBrick[i] = q;
        for (int a = 0; a < 3; ++a) {
            reach[6*q + 2*a] = std::min(reach[6*q + 2*a], c[a]);
            reach[6*q + 2*a + 1] = std::max(reach[6*q + 2*a + 1], c[a]);
        }
    }
    const size_t nPB = particleBrickKey.size();
    std::vector<size_t> firstOf(nPB + 1, 0);
    for (size_t i = 0; i < n; ++i)
        ++firstOf[size_t(particleBrick[i]) + 1];
    for (size_t q = 0; q < nPB; ++q)
        firstOf[q + 1] += firstOf[q];
    std::vector<std::uint32_t> members(n);
    {
        std::vector<size_t> at(firstOf.begin(), firstOf.end() - 1);
        for (size_t i = 0; i < n; ++i)
            members[at[size_t(particleBrick[i])]++] = std::uint32_t(i);
    }

    // The bricks of the field: those the ranges overlap
    KeyMap bricks(27*nPB);
    std::vector<std::int64_t> coord;   // 3 per brick
    for (size_t q = 0; q < nPB; ++q) {
        std::int64_t b0[3], b1[3];
        for (int a = 0; a < 3; ++a) {
            b0[a] = (reach[6*q + 2*a] - KR - 1) >> LogB;
            b1[a] = (reach[6*q + 2*a + 1] + 1 + KR) >> LogB;
        }
        for (std::int64_t z = b0[2]; z <= b1[2]; ++z)
            for (std::int64_t y = b0[1]; y <= b1[1]; ++y)
                for (std::int64_t x = b0[0]; x <= b1[0]; ++x) {
                    const std::int32_t next = std::int32_t(coord.size()/3);
                    if (bricks.insert(pack(x, y, z), next) == next) {
                        coord.push_back(x);
                        coord.push_back(y);
                        coord.push_back(z);
                    }
                }
    }
    const size_t nBricks = coord.size()/3;
    std::vector<std::int32_t> nb(27*nBricks);
    for (size_t q = 0; q < nBricks; ++q)
        for (int dz = -1; dz <= 1; ++dz)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                    nb[27*q + neighbour(dx, dy, dz)] = bricks.find(pack(coord[3*q] + dx, coord[3*q + 1] + dy, coord[3*q + 2] + dz));

    // Trilinear splat, by bricks of particles in eight passes: the bricks
    // of one pass are two apart along some axis, so the bricks they write
    // (their own and those above) are apart too
    std::vector<std::int32_t> homeOf(nPB);
    std::vector<size_t> passes[8];
    for (size_t q = 0; q < nPB; ++q) {
        homeOf[q] = bricks.find(particleBrickKey[q]);
        const std::int64_t *c = &coord[3*size_t(homeOf[q])];
        passes[(c[0] & 1) | (c[1] & 1) << 1 | (c[2] & 1) << 2].push_back(q);
    }
    float *G = fieldBuffer.get(nBricks*B3), *T = blurBuffer.get(nBricks*B3);
    pool->run(nBricks, threads, [&](size_t q) { std::fill_n(&G[q*B3], B3, 0.0f); }, 64);
    for (const std::vector<size_t> &pass : passes)
        pool->run(pass.size(), threads, [&](size_t j) {
            const size_t q = pass[j];
            const std::int32_t *around = &nb[27*size_t(homeOf[q])];
            for (size_t m = firstOf[q]; m < firstOf[q + 1]; ++m) {
                const size_t i = members[m];
                const std::int64_t *c = &node[3*i];
                const float *f = &frac[3*i];
                const int l[3] = {int(c[0] & (B - 1)), int(c[1] & (B - 1)), int(c[2] & (B - 1))};
                for (int dz = 0; dz < 2; ++dz)
                    for (int dy = 0; dy < 2; ++dy)
                        for (int dx = 0; dx < 2; ++dx) {
                            const int x = l[0] + dx, y = l[1] + dy, z = l[2] + dz;
                            const std::int32_t r = around[neighbour(x >> LogB, y >> LogB, z >> LogB)];
                            const float w = (dx ? f[0] : 1 - f[0])*(dy ? f[1] : 1 - f[1])*(dz ? f[2] : 1 - f[2]);
                            G[size_t(r)*B3 + local(x, y, z)] += w;
                        }
            }
        }, 4);

    // Separable Gaussian along x (G to T), y (T to G) and z (G to T, with
    // the normalisation (d/h)^3), zeros outside the bricks: a missing
    // brick adds nothing
    float k[2*KR + 1];
    float ksum = 0;
    for (int j = -KR; j <= KR; ++j)
        ksum += (k[j + KR] = float(std::exp(-0.5*(j/Sigma)*(j/Sigma))));
    for (float &kj : k)
        kj /= ksum;
    const float scale = 8.0f;   // (d/h)^3
    pool->run(nBricks, threads, [&](size_t q) {
        const std::int32_t below = nb[27*q + neighbour(-1, 0, 0)], above = nb[27*q + neighbour(1, 0, 0)];
        blurX(&G[q*B3], below >= 0 ? &G[size_t(below)*B3] : nullptr, above >= 0 ? &G[size_t(above)*B3] : nullptr, &T[q*B3], k);
    }, 16);
    // Along y and z, whole rows along x at once, through the brick and its
    // neighbours along the axis; the last pass keeps the least and greatest
    // value of each brick
    std::vector<float> bmin(nBricks), bmax(nBricks);
    for (int a = 1; a < 3; ++a) {
        const float *src = a == 1 ? T : G;
        float *dst = a == 1 ? G : T;
        const float factor = a == 2 ? scale : 1.0f;
        const int s = a == 1 ? B : B*B, so = a == 1 ? B*B : B;
        pool->run(nBricks, threads, [&](size_t q) {
            const std::int32_t below = nb[27*q + neighbour(0, a == 1 ? -1 : 0, a == 2 ? -1 : 0)];
            const std::int32_t above = nb[27*q + neighbour(0, a == 1, a == 2)];
            float range[2];
            blurYZ(&src[q*B3], below >= 0 ? &src[size_t(below)*B3] : nullptr, above >= 0 ? &src[size_t(above)*B3] : nullptr,
                &dst[q*B3], k, s, so, factor, a == 2 ? range : nullptr);
            if (a == 2) {
                bmin[q] = range[0];
                bmax[q] = range[1];
            }
        }, 16);
    }
    const float *field = T;

    // The bricks the level crosses: those whose values, with the nodes of
    // the bricks above, do not all lie on one side of it
    std::vector<size_t> crossed;
    std::vector<std::int32_t> crossedIndex(nBricks, -1);
    for (size_t q = 0; q < nBricks; ++q) {
        float vmin = bmin[q], vmax = bmax[q];
        for (int c = 1; c < 8; ++c) {
            const std::int32_t r = nb[27*q + neighbour(c & 1, (c >> 1) & 1, c >> 2)];
            vmin = std::min(vmin, r >= 0 ? bmin[size_t(r)] : 0.0f);
            vmax = std::max(vmax, r >= 0 ? bmax[size_t(r)] : 0.0f);
        }
        if (vmax > Iso && vmin <= Iso) {
            crossedIndex[q] = std::int32_t(crossed.size());
            crossed.push_back(q);
        }
    }
    const size_t nCrossed = crossed.size();
    mesh.bricks = nCrossed;
    if (mesh.vertices.size() < nCrossed) {
        mesh.vertices.resize(nCrossed);
        mesh.triangles.resize(nCrossed);
    }

    // The vertices: one per crossed grid edge, made by the brick of the
    // edge's lower node; the table gives each edge's vertex in its brick
    // (entries only for crossed edges, the only ones read)
    std::int32_t *table = tableBuffer.get(nCrossed*3*B3);
    pool->run(nCrossed, threads, [&](size_t a) {
        const size_t q = crossed[a];
        float cube[C*C*C];
        unsigned inside[C*C];
        gather(field, &nb[27*q], q, cube, inside);
        std::vector<float> &out = mesh.vertices[a];
        out.clear();
        std::int32_t *tab = &table[a*3*B3];
        const std::int64_t origin[3] = {coord[3*q]*B, coord[3*q + 1]*B, coord[3*q + 2]*B};
        for (int z = 0; z < B; ++z)
            for (int y = 0; y < B; ++y) {
                const unsigned row = inside[y + C*z];
                const unsigned along[3] = {(row ^ (row >> 1)), row ^ inside[y + 1 + C*z], row ^ inside[y + C*(z + 1)]};
                for (int x = 0; x < B; ++x)
                    for (int ax = 0; ax < 3; ++ax) {
                        if (!((along[ax] >> x) & 1))
                            continue;
                        const float v0 = cube[x + C*y + C*C*z];
                        const float v1 = cube[x + (ax == 0) + C*(y + (ax == 1)) + C*C*(z + (ax == 2))];
                        const double t = double((Iso - v0)/(v1 - v0));
                        const double at[3] = {double(origin[0] + x), double(origin[1] + y), double(origin[2] + z)};
                        tab[3*local(x, y, z) + ax] = std::int32_t(out.size()/3);
                        for (int c = 0; c < 3; ++c)
                            out.push_back(float(lo[c] + (at[c] + (c == ax ? t : 0.0))*h));
                    }
            }
    }, 4);
    mesh.vertexStart.assign(nCrossed + 1, 0);
    for (size_t a = 0; a < nCrossed; ++a)
        mesh.vertexStart[a + 1] = mesh.vertexStart[a] + mesh.vertices[a].size()/3;

    // The polygons of the crossed cubes, fanned into triangles
    pool->run(nCrossed, threads, [&](size_t a) {
        const size_t q = crossed[a];
        const std::int32_t *around = &nb[27*q];
        float cube[C*C*C];
        unsigned inside[C*C];
        gather(field, around, q, cube, inside);
        std::vector<std::uint32_t> &out = mesh.triangles[a];
        out.clear();
        const int cornerOffset[8] = {0, 1, C, 1 + C, C*C, 1 + C*C, C + C*C, 1 + C + C*C};
        for (int iz = 0; iz < B; ++iz)
            for (int iy = 0; iy < B; ++iy) {
                // A row of cubes whose four rows of nodes are all inside or
                // all outside has no crossing, nor has a cube whose corners
                // agree
                const unsigned r0 = inside[iy + C*iz], r1 = inside[iy + 1 + C*iz],
                    r2 = inside[iy + C*(iz + 1)], r3 = inside[iy + 1 + C*(iz + 1)];
                const unsigned all = r0 & r1 & r2 & r3, any = r0 | r1 | r2 | r3;
                if (any == 0 || all == (1u << C) - 1)
                    continue;
                for (int ix = 0; ix < B; ++ix) {
                    if (((any >> ix) & 3) == 0 || ((all >> ix) & 3) == 3)
                        continue;
                    const int base = ix + C*iy + C*C*iz;
                    float v[8];
                    bool in[8];
                    for (int c = 0; c < 8; ++c) {
                        v[c] = cube[base + cornerOffset[c]];
                        in[c] = v[c] > Iso;
                    }

                    // The vertex of a cube edge, from the table of the brick
                    // of its lower node (crossed: the level crosses the edge)
                    auto vertexOf = [&](int e) -> std::int64_t {
                        const int c0 = CubeEdges.low[e], ax = CubeEdges.axis[e];
                        const int x = ix + (c0 & 1), y = iy + ((c0 >> 1) & 1), z = iz + ((c0 >> 2) & 1);
                        const std::int32_t r = around[neighbour(x >> LogB, y >> LogB, z >> LogB)];
                        const std::int32_t owner = r >= 0 ? crossedIndex[size_t(r)] : -1;
                        if (owner < 0)
                            return -1;
                        return std::int64_t(mesh.vertexStart[size_t(owner)]) + table[size_t(owner)*3*B3 + 3*size_t(local(x, y, z)) + ax];
                    };

                    // Each face joins its crossings in pairs, from where the
                    // level enters the inside (counter-clockwise) to where
                    // it leaves; four crossings are paired around the inside
                    // corners or across the inside centre
                    int next[12];
                    for (int e = 0; e < 12; ++e)
                        next[e] = -1;
                    for (int f = 0; f < 6; ++f) {
                        int ce[4];
                        bool entry[4];
                        int nc = 0;
                        for (int s = 0; s < 4; ++s) {
                            const int c1 = FaceCorners[f][s], c2 = FaceCorners[f][(s + 1) % 4];
                            if (in[c1] != in[c2]) {
                                ce[nc] = CubeEdges.id[c1][c2];
                                entry[nc] = in[c2];
                                ++nc;
                            }
                        }
                        if (nc == 2) {
                            next[entry[0] ? ce[0] : ce[1]] = entry[0] ? ce[1] : ce[0];
                        } else if (nc == 4) {
                            float centre = 0;
                            for (int s = 0; s < 4; ++s)
                                centre += v[FaceCorners[f][s]];
                            const bool joined = centre/4 > Iso;
                            for (int r = 0; r < 4; ++r)
                                if (entry[r])
                                    next[ce[r]] = ce[joined ? (r + 3) % 4 : (r + 1) % 4];
                        }
                    }

                    // The loops of crossings, fanned into triangles
                    bool done[12] = {false};
                    for (int e = 0; e < 12; ++e) {
                        if (next[e] < 0 || done[e])
                            continue;
                        std::int64_t loop[12];
                        int m = 0;
                        bool whole = true;
                        int cur = e;
                        while (cur >= 0 && !done[cur] && m < 12) {
                            done[cur] = true;
                            loop[m] = vertexOf(cur);
                            whole = whole && loop[m] >= 0;
                            ++m;
                            cur = next[cur];
                        }
                        if (!whole)
                            continue;
                        for (int r = 1; r + 1 < m; ++r) {
                            out.push_back(std::uint32_t(loop[0]));
                            out.push_back(std::uint32_t(loop[r]));
                            out.push_back(std::uint32_t(loop[r + 1]));
                        }
                    }
                }
            }
    }, 4);
    mesh.triangleStart.assign(nCrossed + 1, 0);
    for (size_t a = 0; a < nCrossed; ++a)
        mesh.triangleStart[a + 1] = mesh.triangleStart[a] + mesh.triangles[a].size()/3;
}

// The thread count: the third input, or MATLAB's maxNumCompThreads
unsigned threadCount(int nrhs, const mxArray *prhs[])
{
    double value = 1;
    if (nrhs > 2) {
        value = mxGetScalar(prhs[2]);
    } else {
        mxArray *out = nullptr;
        if (mexCallMATLAB(1, &out, 0, nullptr, "maxNumCompThreads") == 0 && out != nullptr) {
            value = mxGetScalar(out);
            mxDestroyArray(out);
        }
    }
    if (!(value >= 1))
        return 1;
    return unsigned(std::min(value, 64.0));
}

} // namespace

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 2 && nrhs != 3)
        mexErrMsgIdAndTxt("phx:Liquid:invalidInputs", "Two or three inputs required: P, d and threads.");
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || (mxGetM(prhs[0]) != 3 && !mxIsEmpty(prhs[0])))
        mexErrMsgIdAndTxt("phx:Liquid:invalidSize", "P must be a real 3xN double array.");
    const size_t n = mxIsEmpty(prhs[0]) ? 0 : mxGetN(prhs[0]);
    const double *P = mxGetDoubles(prhs[0]);
    const double d = mxGetScalar(prhs[1]);
    if (!(d > 0))
        mexErrMsgIdAndTxt("phx:Liquid:invalidSpacing", "d must be positive.");
    const unsigned threads = threadCount(nrhs, prhs);
    if (pool == nullptr) {
        pool = new Pool;
        mexAtExit(endPool);
    }

    // The mesh is kept between calls, for the memory of its lists
    static Mesh mesh;
    mesh.bricks = 0;
    mesh.vertexStart.assign(1, 0);
    mesh.triangleStart.assign(1, 0);
    if (n > 0)
        surface(P, n, d, threads, mesh);
    const size_t nCrossed = mesh.bricks;
    const size_t nv = mesh.vertexStart.back(), nf = mesh.triangleStart.back();

    // Outputs in the types a TriangleStrip takes
    plhs[0] = mxCreateUninitNumericMatrix(3, nv, mxSINGLE_CLASS, mxREAL);
    float *V = mxGetSingles(plhs[0]);
    std::uint32_t *Fo = nullptr;
    if (nlhs > 1) {
        plhs[1] = mxCreateUninitNumericMatrix(3, nf, mxUINT32_CLASS, mxREAL);
        Fo = reinterpret_cast<std::uint32_t *>(mxGetUint32s(plhs[1]));
    }
    pool->run(nCrossed, threads, [&](size_t a) {
        std::copy(mesh.vertices[a].begin(), mesh.vertices[a].end(), V + 3*mesh.vertexStart[a]);
        if (Fo != nullptr) {
            std::uint32_t *f = Fo + 3*mesh.triangleStart[a];
            for (std::uint32_t id : mesh.triangles[a])
                *f++ = id + 1;
        }
    }, 16);

    // The normals: the area-weighted sums of the normals of the faces
    // around each vertex
    if (nlhs > 2) {
        plhs[2] = mxCreateNumericMatrix(3, nv, mxSINGLE_CLASS, mxREAL);
        float *N = mxGetSingles(plhs[2]);
        for (size_t a = 0; a < nCrossed; ++a) {
            const std::vector<std::uint32_t> &tri = mesh.triangles[a];
            for (size_t t = 0; t < tri.size(); t += 3) {
                const float *p0 = V + 3*size_t(tri[t]), *p1 = V + 3*size_t(tri[t + 1]), *p2 = V + 3*size_t(tri[t + 2]);
                const float e1[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
                const float e2[3] = {p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]};
                const float fn[3] = {e1[1]*e2[2] - e1[2]*e2[1], e1[2]*e2[0] - e1[0]*e2[2], e1[0]*e2[1] - e1[1]*e2[0]};
                for (int corner = 0; corner < 3; ++corner)
                    for (int c = 0; c < 3; ++c)
                        N[3*size_t(tri[t + corner]) + c] += fn[c];
            }
        }
        const size_t chunk = 4096;
        pool->run((nv + chunk - 1)/chunk, threads, [&](size_t b) {
            for (size_t q = b*chunk; q < std::min(nv, (b + 1)*chunk); ++q) {
                float *nq = N + 3*q;
                const float len = std::max(std::sqrt(nq[0]*nq[0] + nq[1]*nq[1] + nq[2]*nq[2]), 1.17549435e-38f);
                for (int c = 0; c < 3; ++c)
                    nq[c] /= len;
            }
        }, 4);
    }
}
