classdef tBuoyancy < PhxTestCase
%tBuoyancy Volume sampling, buoyancy kernel and floating-equilibrium tests for phx.Buoyancy.
%
%   The voxelization layer is pure geometry (phx.internal.Geometry), so
%   those tests need neither the physics engine nor a graphics session.
%   Neither do the tests of the per-step kernels phx.internal.computeBuoyancy
%   (submerged volume and local center of buoyancy of every body) and
%   phx.internal.computeBuoyancyXY (world x, y of the sampling points for a
%   LevelFunction), checked against hand-computed values and an independent
%   per-body evaluation.
%   The floating tests exercise the full force pipeline end-to-end (a light
%   sphere settles half submerged, a dense body sinks) on headless bodies,
%   so they carry the "Engine" tag and are assumed away when the MEX is
%   absent.
%
%   See also phx.Buoyancy, phx.internal.Geometry

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function voxelBoxFillsItsBoundingBox(tc)
            % Every grid cell center of the bounding box of a box lies
            % inside it, and the represented volume is exact.
            sz = [2 3 4];
            [V, ~, F] = phx.internal.Geometry.triBox(sz);
            [p, dV] = phx.internal.Geometry.voxelize(V, F, 6);
            tc.verifySize(p, [6^3 3]);
            tc.verifyEqual(size(p, 1)*dV, prod(sz), "RelTol", 1e-12);
        end

        function voxelSphereVolumeMatchesAnalytic(tc)
            % The sampled volume equals the mesh volume by construction and
            % the icosphere mesh volume is close to the analytic sphere.
            r = 0.5;
            [V, ~, F] = phx.internal.Geometry.icosphere(3);
            V = V*r;
            [p, dV] = phx.internal.Geometry.voxelize(V, F, 10);
            meshVolume = phx.internal.Geometry.meshMass(V, F, 1);
            tc.verifyEqual(size(p, 1)*dV, meshVolume, "RelTol", 1e-12);
            tc.verifyEqual(meshVolume, 4/3*pi*r^3, "RelTol", 0.05);
            % Interior points are centered on the sphere center
            tc.verifyEqual(mean(p, 1), [0 0 0], "AbsTol", 0.02);
        end

        function voxelizationIsDeterministic(tc)
            [V, ~, F] = phx.internal.Geometry.icosphere(3);
            [p1, dV1] = phx.internal.Geometry.voxelize(V, F, 8);
            [p2, dV2] = phx.internal.Geometry.voxelize(V, F, 8);
            tc.verifyEqual(p1, p2);
            tc.verifyEqual(dV1, dV2);
        end

        function kernelSplitsAtFlatLevel(tc)
            % Points below the level count, the center of buoyancy is their
            % volume-weighted centroid
            [P, dV] = tBuoyancy.fourPoints;
            [Vsub, Vtot, C] = phx.internal.computeBuoyancy(tBuoyancy.columns(eye(4)), P, dV, 4, 0);
            tc.verifyEqual(Vsub, 3, "AbsTol", 1e-14);
            tc.verifyEqual(Vtot, 10, "AbsTol", 1e-14);
            tc.verifyEqual(C, [5/3; 0; -2/3], "AbsTol", 1e-14);
        end

        function kernelTestsWorldHeightReturnsLocalCenter(tc)
            % A quarter turn about y and a lift of 2.5: world z = 2.5 - x,
            % so the two points with local x > 2.5 are under; the center
            % stays in the body frame
            [P, dV] = tBuoyancy.fourPoints;
            T = makehgtform("translate", [5 6 2.5], "yrotate", pi/2);
            [Vsub, Vtot, C] = phx.internal.computeBuoyancy(tBuoyancy.columns(T), P, dV, 4, 0);
            tc.verifyEqual(Vsub, 7, "AbsTol", 1e-14);
            tc.verifyEqual(Vtot, 10, "AbsTol", 1e-14);
            tc.verifyEqual(C, [25/7; 0; 11/14], "AbsTol", 1e-14);
        end

        function kernelPerPointLevel(tc)
            [P, dV] = tBuoyancy.fourPoints;
            [Vsub, ~, C] = phx.internal.computeBuoyancy(tBuoyancy.columns(eye(4)), P, dV, 4, [0.5 -1 0 2]);
            tc.verifyEqual(Vsub, 5, "AbsTol", 1e-14);
            tc.verifyEqual(C, [17/5; 0; 3/5], "AbsTol", 1e-14);
        end

        function kernelConstantLevelVectorMatchesScalar(tc)
            [M, P, dV, counts] = tBuoyancy.randomBodies(4);
            [V1, T1, C1] = phx.internal.computeBuoyancy(M, P, dV, counts, 0.1);
            [V2, T2, C2] = phx.internal.computeBuoyancy(M, P, dV, counts, repmat(0.1, 1, size(P, 2)));
            tc.verifyEqual([V2; T2; C2], [V1; T1; C1]);
        end

        function kernelBodiesAreIndependent(tc)
            % One call over all bodies equals a call per body; a body lifted
            % clear of the liquid has no submerged volume and a zero center
            [M, P, dV, counts] = tBuoyancy.randomBodies(3);
            M(15, 2) = 10;
            [Vsub, Vtot, C] = phx.internal.computeBuoyancy(M, P, dV, counts, 0);
            last = cumsum(counts);
            for a = 1:3
                cols = last(a) - counts(a) + 1:last(a);
                [v, t, c] = phx.internal.computeBuoyancy(M(:, a), P(:, cols), dV(cols), counts(a), 0);
                tc.verifyEqual([Vsub(a); Vtot(a); C(:, a)], [v; t; c]);
            end
            tc.verifyEqual(Vsub(2), 0);
            tc.verifyEqual(C(:, 2), [0; 0; 0]);
            tc.verifyGreaterThan(Vsub([1 3]), 0);
        end

        function kernelMatchesPerBodyReference(tc)
            [M, P, dV, counts] = tBuoyancy.randomBodies(5);
            s = RandStream("twister", "Seed", 7);
            level = 0.2*randn(s, 1, size(P, 2));
            [Vsub, Vtot, C] = phx.internal.computeBuoyancy(M, P, dV, counts, level);

            last = cumsum(counts);
            for a = 1:5
                cols = last(a) - counts(a) + 1:last(a);
                W = reshape(M(:, a), 4, 4)*[P(:, cols); ones(1, counts(a))];
                sub = W(3, :) < level(cols);
                v = dV(cols);
                tc.verifyEqual(Vsub(a), sum(v(sub)), "RelTol", 1e-12);
                tc.verifyEqual(Vtot(a), sum(v), "RelTol", 1e-12);
                tc.verifyEqual(C(:, a), P(:, cols(sub))*v(sub)'/sum(v(sub)), "AbsTol", 1e-12);
            end
        end

        function kernelXYOfRotatedBody(tc)
            % A quarter turn about z maps local x onto world y
            T = makehgtform("translate", [5 6 7], "zrotate", pi/2);
            XY = phx.internal.computeBuoyancyXY(tBuoyancy.columns(T), [1 0; 0 2; 0 3], 2);
            tc.verifyEqual(XY, [5 3; 7 6], "AbsTol", 1e-14);
        end

        function kernelXYMatchesPerBodyReference(tc)
            [M, P, ~, counts] = tBuoyancy.randomBodies(5);
            XY = phx.internal.computeBuoyancyXY(M, P, counts);
            last = cumsum(counts);
            for a = 1:5
                cols = last(a) - counts(a) + 1:last(a);
                W = reshape(M(:, a), 4, 4)*[P(:, cols); ones(1, counts(a))];
                tc.verifyEqual(XY(:, cols), W(1:2, :), "AbsTol", 1e-12);
            end
        end
    end

    methods (Test, TestTags = {'Engine'})
        function halfDensitySphereFloatsHalfSubmerged(tc)
            % A sphere with half the density of the liquid displaces half
            % of its volume, so its center settles at the liquid level. The
            % sampling quantizes the equilibrium into a dead band of half a
            % grid layer (D/2R = 1/32 m here), plus the icosphere mesh
            % volume sits ~3 % under the analytic sphere volume the mass is
            % computed from - together well within the 0.06 m tolerance.
            tc.requireEngine;
            b = tc.spawnBody([0 0 0.2], "Shape", {"Sphere", "Radius", 0.5, "Density", 500});
            phx.Buoyancy(b, "Level", 0, "Resolution", 16, "LinearDamping", 2000, ...
                "AngularDamping", 20, "SurfaceSize", [0 0]);
            sim = phx.Simulation(b);
            tc.addTeardown(@() delete(sim));

            sim.step(6, 1200, -1); % dt = 5 ms

            tc.verifyEqual(b.Position(3), 0, "AbsTol", 0.06);
            tc.verifyLessThan(abs(b.LinearVelocity(3)), 0.05);
        end

        function denserThanLiquidBodySinks(tc)
            tc.requireEngine;
            b = tc.spawnBody([0 0 0.5], "Shape", {"Box", "Size", [0.4 0.4 0.4], "Density", 3000});
            phx.Buoyancy(b, "Level", 0, "LinearDamping", 50, "SurfaceSize", [0 0]);
            sim = phx.Simulation(b);
            tc.addTeardown(@() delete(sim));

            sim.step(2, 400, -1); % dt = 5 ms, no floor to stop the body

            tc.verifyLessThan(b.Position(3), -1);
        end
    end

    methods (Static, Access = private)
        function [P, dV] = fourPoints
            % Local points at z = -1, -0.5, 0.5, 1 with volumes 1..4
            P = [1 2 3 4; 0 0 0 0; -1 -0.5 0.5 1];
            dV = [1 2 3 4];
        end

        function M = columns(varargin)
            % 4x4 transforms as the 16xN columns the kernels take
            M = cell2mat(cellfun(@(T) T(:), varargin, "UniformOutput", false));
        end

        function [M, P, dV, counts] = randomBodies(n)
            % Reproducible bodies with 20..60 points each, tilted and placed
            % around the level z = 0
            s = RandStream("twister", "Seed", n);
            counts = randi(s, [20 60], 1, n);
            P = 0.5*randn(s, 3, sum(counts));
            dV = rand(s, 1, sum(counts))*1e-3;
            M = zeros(16, n);
            for a = 1:n
                T = makehgtform("translate", [2*a 0 0.3*randn(s)], ...
                    "xrotate", randn(s), "yrotate", randn(s), "zrotate", randn(s));
                M(:, a) = T(:);
            end
        end
    end

end
