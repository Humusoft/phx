classdef tRaycast < PhxTestCase
%tRaycast Tests for phx.Raycast ray sensing.
%
%   The configuration tests need only graphics (a body owns an hgtransform)
%   and check what the sensor reports before and between casts: default
%   geometry, the aligned NaN outputs, and the two documented errors. The
%   geometric tests query the engine through a real phx.Simulation - hit
%   points, normals, distances, the anchor-local frame, both sensor modes
%   and the objectID -> phx.Body lookup - and therefore carry the "Engine"
%   tag.
%
%   A sensor is never handed to the simulation itself: only bodies are, and
%   the sensor joins the pipeline as a child of its anchor. So every test
%   here builds its bodies, then the sensor, then the simulation.
%
%   See also phx.Raycast, phx.Zone, phx.Measure

%   Copyright 2026 HUMUSOFT s.r.o.

    properties
        Ax
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.Ax = tc.prepareAxes;
        end
    end

    % --- Configuration and reporting (engine-free) ----------------------
    methods (Test, TestTags = {'Graphics'})
        function defaultIsOneDownwardRay(tc)
            rc = phx.Raycast(tc.mkBody([0 0 0]));
            tc.verifyEqual(rc.Count, 1);
            tc.verifyEqual(rc.Origins, [0; 0; 0]);
            tc.verifyEqual(rc.Ends, [0; 0; -1]);
        end

        function outputsAreAlignedAndEmptyBeforeAnyCast(tc)
            rc = phx.Raycast(tc.mkBody([0 0 0]), "Ends", zeros(3, 4));
            tc.verifyEqual(size(rc.Points), [3 4]);
            tc.verifyEqual(size(rc.Normals), [3 4]);
            tc.verifyEqual(size(rc.Distances), [1 4]);
            tc.verifyEqual(size(rc.Hits), [1 4]);
            tc.verifyTrue(all(isnan(rc.Points(:))));
            tc.verifyTrue(all(isnan(rc.Normals(:))));
            tc.verifyTrue(all(isnan(rc.Distances)));
            tc.verifyFalse(any(rc.Hits));
            tc.verifyEmpty(rc.Bodies);
        end

        function countAndOutputsFollowEnds(tc)
            rc = phx.Raycast(tc.mkBody([0 0 0]), "Ends", zeros(3, 2));
            tc.verifyEqual(rc.Count, 2);
            rc.Ends = zeros(3, 7);
            tc.verifyEqual(rc.Count, 7);
            tc.verifyEqual(size(rc.Points), [3 7]);
            tc.verifyEqual(size(rc.Distances), [1 7]);
            tc.verifyEqual(size(rc.Hits), [1 7]);
        end

        function noRaysIsEmptyNotAnError(tc)
            rc = phx.Raycast(tc.mkBody([0 0 0]), "Ends", zeros(3, 0));
            tc.verifyEqual(rc.Count, 0);
            tc.verifyEmpty(rc.Points);
            tc.verifyEmpty(rc.Hits);
            tc.verifyEmpty(rc.Bodies);
        end

        function anchorIsMandatoryAndMustBeABody(tc)
            tc.verifyError(@() phx.Raycast(), "MATLAB:minrhs");
            tc.verifyError(@() phx.Raycast(42), "MATLAB:validators:mustBeA");
        end

        function updateWithoutSimulationErrors(tc)
            rc = phx.Raycast(tc.mkBody([0 0 0]), "SimulationOrder", "none");
            tc.verifyError(@() rc.update(), "phx:Raycast:noSimulation");
        end
    end

    % --- Geometry, modes and the body lookup ----------------------------
    methods (Test, TestTags = {'Engine', 'Graphics'})
        function pickedUpAsAChildOfItsAnchor(tc)
            % The sensor is never passed to the simulation, so this is what
            % every other engine test here silently relies on.
            tc.requireEngine;
            [~, probe] = tc.probeBodies(5);
            rc = phx.Raycast(probe, "Ends", [0; 0; -10]);
            sim = tc.startSim;

            tc.verifyFalse(any(cellfun(@(c) c == rc, sim.Children)), ...
                "The sensor must not be a direct child of the simulation.");
            sim.step(0.01, 1, -1);
            tc.verifyTrue(rc.Hits, ...
                "A sensor anchored to a body in the simulation must be cast.");
        end

        function reportsHitPointNormalDistanceAndBody(tc)
            tc.requireEngine;
            [floor, probe] = tc.probeBodies(5);
            rc = phx.Raycast(probe, "Ends", [0; 0; -10]);
            sim = tc.startSim;

            sim.step(0.01, 1, -1);
            tc.verifyEqual(rc.Distances, 5, "AbsTol", 1e-9);
            tc.verifyEqual(rc.Points, [0; 0; 0], "AbsTol", 1e-9);
            tc.verifyEqual(rc.Normals, [0; 0; 1], "AbsTol", 1e-9);
            tc.verifyTrue(rc.Hits);
            tc.verifyNumElements(rc.Bodies, 1);
            tc.verifyTrue(rc.Bodies(1) == floor);
        end

        function missIsNaNNotZero(tc)
            tc.requireEngine;
            [~, probe] = tc.probeBodies(5);
            rc = phx.Raycast(probe, "Ends", [0; 0; -1]);   % too short
            sim = tc.startSim;

            sim.step(0.01, 1, -1);
            tc.verifyTrue(isnan(rc.Distances));
            tc.verifyTrue(all(isnan(rc.Points)));
            tc.verifyTrue(all(isnan(rc.Normals)));
            tc.verifyFalse(rc.Hits);
            tc.verifyEmpty(rc.Bodies);
        end

        function raysAreAnchorLocalAndFollowTheBody(tc)
            tc.requireEngine;
            [~, probe] = tc.probeBodies(5);
            rc = phx.Raycast(probe, "Ends", [0; 0; -10]);
            sim = tc.startSim;

            sim.step(0.01, 1, -1);
            tc.verifyEqual(rc.Distances, 5, "AbsTol", 1e-9);

            probe.Position = [0 0 3];                     % translation
            sim.step(0.01, 1, -1);
            tc.verifyEqual(rc.Distances, 3, "AbsTol", 1e-9);

            probe.EulerAngles = [0 pi/2 0];               % rotation
            sim.step(0.01, 1, -1);
            tc.verifyFalse(rc.Hits, ...
                "A ray swung horizontal must no longer reach the floor.");
        end

        function fanGeometryIsExact(tc)
            tc.requireEngine;
            h = 5;
            [~, probe] = tc.probeBodies(h);
            n = 25;
            th = linspace(-pi/4, pi/4, n);
            rc = phx.Raycast(probe, "Origins", [0; 0; 0], ...
                "Ends", [20*sin(th); zeros(1, n); -20*cos(th)]);
            sim = tc.startSim;

            sim.step(0.01, 1, -1);
            tc.verifyTrue(all(rc.Hits));
            tc.verifyEqual(rc.Points(3, :), zeros(1, n), "AbsTol", 1e-9);
            tc.verifyEqual(rc.Points(1, :), h*tan(th), "AbsTol", 1e-6);
            tc.verifyEqual(rc.Distances, h./cos(th), "AbsTol", 1e-6);
            tc.verifyNumElements(rc.Bodies, nnz(rc.Hits));
        end

        function perRayOriginsAreHonoured(tc)
            tc.requireEngine;
            [~, probe] = tc.probeBodies(5);
            n = 9;
            x = linspace(-2, 2, n);
            rc = phx.Raycast(probe, ...
                "Origins", [x; zeros(1, n); zeros(1, n)], ...
                "Ends",    [x; zeros(1, n); -10*ones(1, n)]);
            sim = tc.startSim;

            sim.step(0.01, 1, -1);
            tc.verifyTrue(all(rc.Hits));
            tc.verifyEqual(rc.Points(1, :), x, "AbsTol", 1e-9);
            tc.verifyEqual(rc.Distances, 5*ones(1, n), "AbsTol", 1e-9);
        end

        function hitsAndMissesStayColumnAligned(tc)
            tc.requireEngine;
            [floor, probe] = tc.probeBodies(5);
            rc = phx.Raycast(probe, ...
                "Ends", [0 0 0 0; 0 0 0 0; -10 10 -10 10]);   % down/up/down/up
            sim = tc.startSim;

            sim.step(0.01, 1, -1);
            tc.verifyEqual(rc.Hits, [true false true false]);
            tc.verifyEqual(isnan(rc.Distances), [false true false true]);
            tc.verifyNumElements(rc.Bodies, 2);
            tc.verifyTrue(all(rc.Bodies == floor));
        end

        function mismatchedOriginsError(tc)
            tc.requireEngine;
            [~, probe] = tc.probeBodies(5);
            phx.Raycast(probe, "Ends", zeros(3, 3), "Origins", zeros(3, 5));
            sim = tc.startSim;
            tc.verifyError(@() sim.step(0.01, 1, -1), "phx:Raycast:sizeMismatch");
        end

        function activeSensorCastsEveryStep(tc)
            tc.requireEngine;
            [~, probe] = tc.probeBodies(5);
            rc = phx.Raycast(probe, "Ends", [0; 0; -10]);
            sim = tc.startSim;

            sim.step(0.01, 1, -1);
            tc.verifyEqual(rc.Distances, 5, "AbsTol", 1e-9);
            probe.Position = [0 0 7];
            sim.step(0.01, 1, -1);
            tc.verifyEqual(rc.Distances, 7, "AbsTol", 1e-9);
        end

        function manualSensorOnlyCastsOnUpdate(tc)
            tc.requireEngine;
            [~, probe] = tc.probeBodies(5);
            rc = phx.Raycast(probe, "Ends", [0; 0; -10], "SimulationOrder", "none");
            sim = tc.startSim;

            sim.step(0.01, 1, -1);
            tc.verifyTrue(isnan(rc.Distances), ...
                "A manual sensor must not be cast by step().");
            rc.update();
            tc.verifyEqual(rc.Distances, 5, "AbsTol", 1e-9);

            probe.Position = [0 0 7];
            sim.step(0.01, 1, -1);
            tc.verifyEqual(rc.Distances, 5, "AbsTol", 1e-9, ...
                "A manual sensor must keep its last value across a step.");
            rc.update();
            tc.verifyEqual(rc.Distances, 7, "AbsTol", 1e-9);
        end

        function updateAcceptsAnArray(tc)
            tc.requireEngine;
            [~, probe] = tc.probeBodies(4);
            a = phx.Raycast(probe, "Ends", [0; 0; -10], "SimulationOrder", "none");
            b = phx.Raycast(probe, "Ends", [0; 0; -10], "SimulationOrder", "none");
            tc.startSim;

            update([a b]);
            tc.verifyEqual([a.Distances b.Distances], [4 4], "AbsTol", 1e-9);
        end

        function bodyLookupSurvivesARebuild(tc)
            % The objectID -> phx.Body lookup is resolved lazily. A body
            % added AFTER the sensor must still resolve: at initObject time
            % its engine handle does not exist yet.
            tc.requireEngine;
            [floor, probe] = tc.probeBodies(8);
            rc = phx.Raycast(probe, "Ends", [0; 0; -20]);
            sim = tc.startSim;

            sim.step(0.01, 1, -1);
            tc.verifyTrue(rc.Bodies(1) == floor);

            slab = phx.Body(tc.Ax, "Type", "static", "Position", [0 0 3], ...
                "Shape", {"Box", "Size", [4 4 2]});
            sim.addObjects(slab);
            sim.step(0.01, 1, -1);
            tc.verifyEqual(rc.Distances, 4, "AbsTol", 1e-6);
            tc.verifyNumElements(rc.Bodies, 1);
            tc.verifyTrue(rc.Bodies(1) == slab, ...
                "A body added after the sensor must resolve in Bodies.");

            delete(slab);
            sim.step(0.01, 1, -1);
            tc.verifyEqual(rc.Distances, 8, "AbsTol", 1e-6);
            tc.verifyTrue(rc.Bodies(1) == floor);
            tc.verifyTrue(all(isvalid(rc.Bodies)));
        end

        function changingEndsVoidsTheStoredResults(tc)
            tc.requireEngine;
            [~, probe] = tc.probeBodies(5);
            rc = phx.Raycast(probe, "Ends", [0; 0; -10]);
            sim = tc.startSim;

            sim.step(0.01, 1, -1);
            tc.verifyTrue(rc.Hits);

            rc.Ends = zeros(3, 3);      % a different ray set: columns no
            tc.verifyEqual(size(rc.Points), [3 3]);   % longer correspond
            tc.verifyTrue(all(isnan(rc.Distances)));
            tc.verifyFalse(any(rc.Hits));
            tc.verifyEmpty(rc.Bodies);
        end

        function sensorDoesNotPerturbThePhysics(tc)
            tc.requireEngine;
            pWithout = tc.fallRun(false);
            pWith = tc.fallRun(true);
            tc.verifyEqual(pWith, pWithout, ...
                "Casting rays must not change the trajectory at all.");
        end

        function teardownWithSensorIsClean(tc)
            % Tearing the world down while a sensor is attached must not warn
            % from the destructor: phx.Simulation.destroyObject walks its
            % children, and a body carrying a sensor must survive that.
            tc.requireEngine;
            [floor, probe] = tc.probeBodies(5);
            rc = phx.Raycast(probe, "Ends", [0; 0; -10]);
            sim = tc.startSim;
            sim.step(0.01, 1, -1);

            tc.verifyWarningFree(@() delete(sim));
            tc.verifyFalse(isvalid(sim));
            tc.verifyTrue(isvalid(floor));
            tc.verifyEmpty(floor.ObjectHandle);
            tc.verifyTrue(isvalid(rc));
        end
    end

    methods (Access = private)
        function b = mkBody(tc, pos)
            b = phx.Body(tc.Ax, "Position", pos, "Type", "static");
        end

        function [floor, probe] = probeBodies(tc, height)
            % Floor with its top face at z = 0 and a static probe body
            % hovering the given height above it. No simulation yet - the
            % sensor has to exist before one is built.
            floor = phx.Body(tc.Ax, "Type", "static", "Position", [0 0 -0.5], ...
                "Shape", {"Box", "Size", [40 40 1]});
            probe = phx.Body(tc.Ax, "Type", "static", "Position", [0 0 height], ...
                "Shape", {"Sphere", "Diameter", 0.1});
        end

        function sim = startSim(tc)
            % Build the simulation from the axes, which collects the bodies
            % and reaches their children - the sensor among them.
            sim = phx.Simulation(tc.Ax, "Gravity", [0 0 0]);
            tc.addTeardown(@() tc.deleteIfValid(sim));
        end

        function p = fallRun(tc, withSensor)
            ax = axes(figure("Visible", "off"));
            tc.addTeardown(@() close(ancestor(ax, "figure")));
            phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], ...
                "Shape", {"Box", "Size", [40 40 1]});
            ball = phx.Body(ax, "Position", [3 2 6], "Mass", 1, ...
                "Inertia", [1 1 1], "Shape", {"Sphere", "Diameter", 1});
            if withSensor
                phx.Raycast(ball, "Ends", [0; 0; -10]);
            end
            sim = phx.Simulation(ax);
            for k = 1:60
                sim.step(0.005, 1, -1);
            end
            p = ball.Position;
            delete(sim);
        end

        function deleteIfValid(~, obj)
            if isvalid(obj)
                delete(obj);
            end
        end
    end

end
