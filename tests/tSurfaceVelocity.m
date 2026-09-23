classdef tSurfaceVelocity < PhxTestCase
%tSurfaceVelocity Conveyor-belt surface drive of phx.Body.
%
%   SurfaceVelocity gives a body a virtual surface motion: the body stays
%   where it is, but contacts with it are solved as if its surface were
%   sliding, so resting bodies are dragged along. The tests lock in what
%   that means - the commanded speed is reached, the vector is read in the
%   body frame, friction is the only path the drive takes, and the pull is
%   an ordinary contact that pushes back on the belt.
%
%   The validation test only needs a body, the drive tests step the engine.
%
%   See also phx.Body, tBodyKinematics, tForceApplication

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test, TestTags = {'Graphics'})
        function rejectsUnusableVelocity(tc)
            % The property declares a finite 1x3 velocity; anything else
            % would travel straight into the engine.
            b = tc.spawnBody([0 0 0]);
            tc.verifyError(@() set(b, "SurfaceVelocity", [NaN 0 0]), ...
                "MATLAB:validators:mustBeFinite");
            tc.verifyError(@() set(b, "SurfaceVelocity", [Inf 0 0]), ...
                "MATLAB:validators:mustBeFinite");
            tc.verifyError(@() set(b, "SurfaceVelocity", "fast"), ...
                "MATLAB:validators:mustBeFinite");
            tc.verifyError(@() set(b, "SurfaceVelocity", [1 0]), ...
                "MATLAB:validation:IncompatibleSize");

            b.SurfaceVelocity = [-2.5 0 1];
            tc.verifyEqual(b.SurfaceVelocity, [-2.5 0 1]);
        end
    end

    methods (Test, TestTags = {'Engine'})
        function cargoReachesTheCommandedSpeed(tc)
            % Sliding cargo settles at the surface velocity of the belt,
            % whichever way the belt runs. The two default 0.5 friction
            % coefficients combine into 0.25, so the cargo picks up about
            % 2.45 m/s per second and the fastest case here needs 1.3 s.
            tc.requireEngine;
            for commanded = {[1 0 0], [3 0 0], [-2 0 0], [0 1.5 0], [1 1 0]}
                [belt, cargo] = tc.conveyorParts("SurfaceVelocity", commanded{1});
                tc.runConveyor([belt cargo], 2);
                tc.verifyEqual(cargo.LinearVelocity, commanded{1}, "AbsTol", 0.03, ...
                    "Cargo did not reach the commanded belt speed.");
            end
        end

        function velocityIsGivenInTheBodyFrame(tc)
            % A turned belt drives along its own x axis, not along the world
            % one. EulerAngles are in radians.
            tc.requireEngine;
            for yaw = [0 30 90 135]
                [belt, cargo] = tc.conveyorParts("EulerAngles", [0 0 deg2rad(yaw)], ...
                    "SurfaceVelocity", [1 0 0]);
                tc.runConveyor([belt cargo], 1);
                v = cargo.LinearVelocity;
                tc.verifyEqual(atan2d(v(2), v(1)), yaw, "AbsTol", 1, ...
                    "Drive direction does not follow the turned belt.");
                tc.verifyEqual(norm(v), 1, "AbsTol", 0.03);
            end
        end

        function drivePullsOnlyThroughFriction(tc)
            % The surface drive is an ordinary contact constraint, so a
            % frictionless belt transmits nothing however fast it runs.
            tc.requireEngine;
            [belt, cargo] = tc.conveyorParts("SurfaceVelocity", [5 0 0], ...
                "Friction", [0 0 0]);
            tc.runConveyor([belt cargo], 1);
            tc.verifyLessThan(norm(cargo.LinearVelocity), 0.01, ...
                "A frictionless belt moved the cargo.");
        end

        function drivingCargoPushesBackOnTheBelt(tc)
            % The drive is a contact, not a free force: a belt left free to
            % move recoils, and the momentum of the pair stays put. The
            % belt rides on a frictionless floor so that nothing else can
            % take the reaction.
            tc.requireEngine;
            floor = tc.spawnBody([0 0 -1], "Type", "static", ...
                "Shape", {"Box", "Size", [40 40 0.2]}, "Friction", [0 0 0]);
            belt = tc.spawnBody([0 0 -0.8], "Mass", 5, ...
                "Shape", {"Box", "Size", [8 2 0.2]}, "SurfaceVelocity", [1 0 0]);
            cargo = tc.spawnBody([0 0 -0.64], "Mass", 1, ...
                "Shape", {"Box", "Size", [0.2 0.2 0.2]});
            tc.runConveyor([floor belt cargo], 2);

            tc.verifyGreaterThan(cargo.LinearVelocity(1), 0.1, ...
                "The cargo was not driven at all.");
            tc.verifyLessThan(belt.LinearVelocity(1), 0, ...
                "The belt did not recoil against the cargo it drives.");
            tc.verifyEqual(cargo.Mass*cargo.LinearVelocity(1) + ...
                belt.Mass*belt.LinearVelocity(1), 0, "AbsTol", 0.01, ...
                "The surface drive created momentum out of nothing.");
        end
    end

    methods (Access = private)
        function [belt, cargo] = conveyorParts(tc, varargin)
            % A wide static belt plate in the xy plane with a 0.2 m cube
            % resting on it. Name-value pairs are passed to the belt.
            belt = tc.spawnBody([0 0 0], "Type", "static", ...
                "Shape", {"Box", "Size", [40 40 0.2]}, varargin{:});
            cargo = tc.spawnBody([0 0 0.16], "Mass", 1, ...
                "Shape", {"Box", "Size", [0.2 0.2 0.2]});
        end

        function sim = runConveyor(tc, parts, seconds)
            % Step the given parts headlessly at dt = 5 ms.
            sim = phx.Simulation(parts);
            tc.addTeardown(@() delete(sim));
            sim.step(seconds, seconds/0.005, -1);
        end
    end

end
