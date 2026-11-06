classdef tCenterOfMass < PhxTestCase
%tCenterOfMass Tests of the centre of mass of phx.Body.
%
%   The shape gives a body its CenterOfMass as it gives Mass and Inertia; a
%   value given after the shape stays until a new shape brings its own. The
%   engine cases check that it is the point the body turns about and gravity
%   acts at, and that changing it during the run does not move the body.
%
%   See also phx.Body, tShapeMass, tEngineProtocol

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function shapeGivesTheCentre(tc)
            b = tc.spawnBody([0 0 0], "Shape", {"Cone", "Diameter", 1, "Height", 1});
            tc.verifyEqual(b.CenterOfMass, [0 0 -0.25], "AbsTol", 1e-12);
            b = tc.spawnBody([0 0 0], "Shape", {"Cone", "Height", 1, "Axis", "x"});
            tc.verifyEqual(b.CenterOfMass, [-0.25 0 0], "AbsTol", 1e-12);
            b = tc.spawnBody([0 0 0]);
            tc.verifyEqual(b.CenterOfMass, [0 0 0], "The default box has it at the origin.");
        end

        function givenCentreStaysUntilTheShapeChanges(tc)
            b = tc.spawnBody([0 0 0], "Shape", {"Box"}, "CenterOfMass", [0.1 0 0]);
            tc.verifyEqual(b.CenterOfMass, [0.1 0 0]);
            b = tc.spawnBody([0 0 0], "CenterOfMass", [0.1 0 0]);
            tc.verifyEqual(b.CenterOfMass, [0.1 0 0], "The default shape keeps a given centre.");
            b.Shape = {"Cone"};
            tc.verifyEqual(b.CenterOfMass, [0 0 -0.25], "AbsTol", 1e-12);
            tc.verifyError(@() setCentre(b, [NaN 0 0]), "MATLAB:validators:mustBeFinite");
        end
    end

    methods (Test, TestTags = {'Engine'})
        function bodyTurnsAboutItsCentre(tc)
            % Spinning with its centre at rest, a body whose centre lies
            % 0.3 m off the origin takes the origin 0.6 m across in half a turn
            tc.requireEngine;
            b = tc.spawnBody([0 0 1], "CenterOfMass", [0.3 0 0]);
            sim = phx.Simulation(b, "Gravity", [0 0 0]);
            tc.addTeardown(@() delete(sim));
            b.AngularVelocity = [0 0 pi];
            sim.step(1, 200, -1);
            tc.verifyEqual(b.Position, [0.6 0 1], "AbsTol", 1e-3);
        end

        function centreChangedDuringTheRunKeepsThePose(tc)
            % The new centre reaches the engine without moving the body;
            % from then on the body turns about it
            tc.requireEngine;
            b = tc.spawnBody([0 0 1]);
            sim = phx.Simulation(b, "Gravity", [0 0 0]);
            tc.addTeardown(@() delete(sim));
            sim.step(0.1, 10, -1);
            b.CenterOfMass = [0.2 0 0];
            sim.step(0.1, 10, -1);
            tc.verifyEqual(b.Position, [0 0 1], "AbsTol", 1e-9);
            b.AngularVelocity = [0 0 pi];
            sim.step(1, 200, -1);
            tc.verifyEqual(b.Position, [0.4 0 1], "AbsTol", 1e-3);
        end

        function boxTipsOverWithItsCentreOutsideTheFootprint(tc)
            % Gravity acts at the centre of mass: a 1 m wide slab stands with
            % its centre inside its footprint and tips over with it outside
            tc.requireEngine;
            tilt = @(x) tc.slabTilt(x);
            tc.verifyLessThan(tilt(0.4), 1);
            tc.verifyGreaterThan(tilt(0.6), 20);
        end
    end

    methods (Access = private)
        function deg = slabTilt(tc, x)
            % Tilt (deg) of a slab 1 x 1 x 0.2 m standing on a floor for
            % 1 s, its centre of mass x off the middle
            floor = tc.spawnBody([0 0 -0.5], "Type", "static", "Shape", {"Box", "Size", [4 4 1]});
            slab = tc.spawnBody([0 0 0.1], "Shape", {"Box", "Size", [1 1 0.2]}, ...
                "CenterOfMass", [x 0 0]);
            sim = phx.Simulation([floor slab]);
            tc.addTeardown(@() delete(sim));
            sim.step(1, 400, -1);
            deg = acosd(slab.Orientation(3, 3));
        end
    end

end

function setCentre(body, center)
body.CenterOfMass = center;
end
