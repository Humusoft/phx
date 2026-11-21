classdef tBodyMass < PhxTestCase
%tBodyMass The mass properties of phx.Body: Density, Volume, Mass, Inertia.
%
%   The shape gives the geometry (volume, moments per unit density, centre)
%   and the body its density. Writing the mass or the density scales the
%   inertia by the ratio of the masses; writing the inertia changes neither;
%   a new shape keeps the density. The constructor takes the mass properties
%   in a fixed order, whatever the order of its arguments. The pure tests
%   use headless bodies ([] axes); the last one runs the engine.
%
%   See also phx.Body, phx.base.Shape.computeVolume

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function shapeGivesTheGeometryAtTheDensity(tc)
            % A 1x2x3 box of the default density 1000 kg/m^3
            s = phx.shape.Box("Size", [1 2 3]);
            b = tc.spawnBody([0 0 0], "Shape", s);
            [V, J, c] = s.computeVolume;
            tc.verifyEqual(b.Volume, 6, "RelTol", 1e-14);
            tc.verifyEqual(b.Density, 1000, "RelTol", 1e-14);
            tc.verifyEqual(b.Mass, 1000*V, "RelTol", 1e-14);
            tc.verifyEqual(b.Inertia, 1000*J, "RelTol", 1e-14);
            tc.verifyEqual(b.CenterOfMass, c);
        end

        function massScalesTheInertia(tc)
            b = tc.spawnBody([0 0 0], "Shape", {"Box", "Size", [1 2 3]});
            I = b.Inertia;
            b.Mass = 2*b.Mass;
            tc.verifyEqual(b.Inertia, 2*I, "RelTol", 1e-14);
            tc.verifyEqual(b.Density, 2000, "RelTol", 1e-14);
        end

        function densityScalesTheMassAndTheInertia(tc)
            b = tc.spawnBody([0 0 0], "Shape", {"Sphere", "Diameter", 0.5});
            m = b.Mass;
            I = b.Inertia;
            b.Density = 7800;
            tc.verifyEqual(b.Mass, 7.8*m, "RelTol", 1e-14);
            tc.verifyEqual(b.Inertia, 7.8*I, "RelTol", 1e-14);
        end

        function inertiaChangesNeitherMassNorDensity(tc)
            b = tc.spawnBody([0 0 0], "Shape", {"Box", "Size", [1 2 3]});
            m = b.Mass;
            b.Inertia = [1 2 3];
            tc.verifyEqual(b.Mass, m);
            tc.verifyEqual(b.Density, 1000, "RelTol", 1e-14);
        end

        function customInertiaKeepsItsDistribution(tc)
            % A custom inertia (here a hollow rotor) scales with later
            % changes of the mass and of the density
            b = tc.spawnBody([0 0 0], "Shape", {"Cylinder", "Diameter", 0.4, "Height", 0.1});
            b.Inertia = [2 2 4];
            b.Mass = 3*b.Mass;
            tc.verifyEqual(b.Inertia, [6 6 12], "RelTol", 1e-14);
            b.Density = b.Density/2;
            tc.verifyEqual(b.Inertia, [3 3 6], "RelTol", 1e-14);
        end

        function newShapeKeepsTheDensity(tc)
            % A new shape sets the volume, the inertia and the centre anew,
            % at the density the body had
            b = tc.spawnBody([0 0 0], "Density", 500);
            b.Inertia = [9 9 9];
            b.Shape = phx.shape.Cone("Diameter", 1, "Height", 2);
            [V, J, c] = phx.shape.Cone("Diameter", 1, "Height", 2).computeVolume;
            tc.verifyEqual(b.Volume, V, "RelTol", 1e-14);
            tc.verifyEqual(b.Density, 500, "RelTol", 1e-14);
            tc.verifyEqual(b.Inertia, 500*J, "RelTol", 1e-14);
            tc.verifyEqual(b.CenterOfMass, c);
        end

        function defaultBodyIsTheUnitBoxOfWater(tc)
            b = tc.spawnBody([0 0 0]);
            tc.verifyEqual([b.Volume b.Density b.Mass], [1 1000 1000], "RelTol", 1e-14);
            tc.verifyEqual(b.Inertia, [1 1 1]*1000/6, "RelTol", 1e-14);
        end

        function constructorTakesMassPropertiesInAFixedOrder(tc)
            % Shape, then the mass, the inertia and the centre, in every
            % order of the arguments
            args = {{"Shape", {"Box", "Size", [1 2 3]}}, {"Mass", 5}, ...
                {"Inertia", [0.1 0.2 0.3]}, {"CenterOfMass", [0 0 0.1]}};
            orders = perms(1:4);
            for k = 1:size(orders, 1)
                pairs = [args{orders(k, :)}];
                b = phx.Body([], pairs{:});
                tc.verifyEqual(b.Mass, 5, "RelTol", 1e-14, mat2str(orders(k, :)));
                tc.verifyEqual(b.Inertia, [0.1 0.2 0.3], "RelTol", 1e-14, mat2str(orders(k, :)));
                tc.verifyEqual(b.CenterOfMass, [0 0 0.1], mat2str(orders(k, :)));
            end
            b = phx.Body([], "Density", 2000, "Shape", {"Box", "Size", [1 2 3]});
            tc.verifyEqual(b.Mass, 12000, "RelTol", 1e-14);
        end

        function densityAndMassTogetherAreRefused(tc)
            tc.verifyError(@() phx.Body([], "Density", 2000, "Mass", 5), "phx:Body:densityAndMass");
        end

        function shapeDensityOfPHX2WarnsAndIsTaken(tc)
            % A density given to the shape (PHX 2.0) warns and reaches the
            % body, unless the body is given its own
            b = tc.verifyWarning(@() phx.Body([], "Shape", {"Box", "Density", 2500}), "phx:Shape:densityMoved");
            tc.verifyEqual(b.Density, 2500, "RelTol", 1e-14);
            b = tc.verifyWarning(@() phx.Body([], "Density", 800, "Shape", {"Box", "Density", 2500}), ...
                "phx:Shape:densityMoved");
            tc.verifyEqual(b.Density, 800, "RelTol", 1e-14);
            tc.verifyError(@() phx.shape.Box("Density", 0), "phx:Shape:badDensity");
            b = tc.verifyWarningFree(@() phx.Body([], "Shape", {"Box"}, "Density", 2500));
            tc.verifyEqual(b.Density, 2500, "RelTol", 1e-14);
        end

        function massPropertiesArePositive(tc)
            b = tc.spawnBody([0 0 0]);
            tc.verifyError(@() set(b, "Mass", 0), "MATLAB:validators:mustBePositive");
            tc.verifyError(@() set(b, "Mass", -1), "MATLAB:validators:mustBePositive");
            tc.verifyError(@() set(b, "Density", 0), "MATLAB:validators:mustBePositive");
            tc.verifyError(@() set(b, "Inertia", [0 1 1]), "MATLAB:validators:mustBePositive");
            tc.verifyError(@() set(b, "Mass", Inf), "MATLAB:validators:mustBeFinite");
        end

        function flatMeshHasAPositiveVolume(tc)
            % An open, flat mesh (a plane) falls back to a box a thousandth
            % of its largest side thick
            s = phx.shape.Mesh("Vertices", [0 0 0; 2 0 0; 2 1 0; 0 1 0], "Faces", [1 2 3; 1 3 4]);
            b = phx.Body([], "Shape", s);
            tc.verifyEqual(b.Volume, 2*1*0.002, "RelTol", 1e-12);
            tc.verifyGreaterThan(b.Inertia, 0);
        end
    end

    methods (Test, TestTags = {'Engine'})
        function massWrittenDuringTheRunReachesTheEngine(tc)
            % In free space a force gives dv = F dt / m: halving the mass
            % during the run doubles the change
            tc.requireEngine;
            b = tc.spawnBody([0 0 1], "Mass", 2);
            sim = phx.Simulation(b, "Gravity", [0 0 0]);
            tc.addTeardown(@() delete(sim));
            dt = 0.01;
            b.applyForce([10 0 0]);
            sim.step(dt, 1, -1);
            tc.verifyEqual(b.LinearVelocity, [10*dt/2 0 0], "RelTol", 1e-5);
            b.Mass = 1;
            b.LinearVelocity = [0 0 0];
            b.applyForce([10 0 0]);
            sim.step(dt, 1, -1);
            tc.verifyEqual(b.LinearVelocity, [10*dt 0 0], "RelTol", 1e-5);
        end
    end
end
