classdef tBuoyancy < matlab.unittest.TestCase
%tBuoyancy Volume sampling and floating-equilibrium tests for phx.Buoyancy.
%
%   The voxelization layer is pure geometry (phx.internal.Geometry), so
%   those tests need neither the physics engine nor a graphics session.
%   The floating tests exercise the full force pipeline end-to-end (a light
%   sphere settles half submerged, a dense body sinks), so they carry the
%   "Engine" tag and are assumed away when the MEX is absent.
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
    end

    methods (Test, TestTags = {'Engine'})
        function halfDensitySphereFloatsHalfSubmerged(tc)
            % A sphere with half the density of the liquid displaces half
            % of its volume, so its center settles at the liquid level. The
            % sampling quantizes the equilibrium into a dead band of half a
            % grid layer (D/2R = 1/32 m here), plus the icosphere mesh
            % volume sits ~3 % under the analytic sphere volume the mass is
            % computed from - together well within the 0.06 m tolerance.
            b = tc.spawnBody([0 0 0.2], {"Sphere", "Radius", 0.5, "Density", 500});
            phx.Buoyancy(b, "Level", 0, "Resolution", 16, "LinearDamping", 2000, ...
                "AngularDamping", 20, "SurfaceSize", [0 0]);
            sim = phx.Simulation(b);
            tc.addTeardown(@() delete(sim));

            sim.step(6, 1200); % dt = 5 ms

            tc.verifyEqual(b.Position(3), 0, "AbsTol", 0.06);
            tc.verifyLessThan(abs(b.LinearVelocity(3)), 0.05);
        end

        function denserThanLiquidBodySinks(tc)
            b = tc.spawnBody([0 0 0.5], {"Box", "Size", [0.4 0.4 0.4], "Density", 3000});
            phx.Buoyancy(b, "Level", 0, "LinearDamping", 50, "SurfaceSize", [0 0]);
            sim = phx.Simulation(b);
            tc.addTeardown(@() delete(sim));

            sim.step(2, 400); % dt = 5 ms, no floor to stop the body

            tc.verifyLessThan(b.Position(3), -1);
        end
    end

    methods (Access = private)
        function b = spawnBody(tc, position, shape)
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");
            f = figure("Visible", "off");
            tc.addTeardown(@() close(f));
            b = phx.Body(axes(f), "Position", position, "Shape", shape);
        end
    end

end
