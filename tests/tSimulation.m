classdef tSimulation < matlab.unittest.TestCase
%tSimulation Integration tests that exercise the physics engine (MEX).
%
%   These verify end-to-end behavior: a body really falls, a static body
%   really stays put, and repeated runs are bit-for-bit reproducible (the
%   determinism guarantee demonstrated by phxex_determinism). They depend on
%   the engine binary and on graphics, so every test carries the "Engine"
%   tag and can be excluded on machines without the MEX; the draw-path
%   independence test additionally carries the "Graphics" tag.
%
%   See also phx.Simulation, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (TestClassSetup)
        function requireEngine(tc)
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");
        end
    end

    methods (Test, TestTags = {'Engine'})
        function freeFallMatchesAnalyticDrop(tc)
            g = 9.81;
            t = 1.0;
            z0 = 50;                       % start high, no ground to hit
            b = tc.spawnBody([0 0 z0], "dynamic");
            sim = phx.Simulation(b, "Gravity", [0 0 -g]);
            tc.addTeardown(@() delete(sim));

            sim.step(t, 1000);

            tc.verifyEqual(sim.Time, t, "AbsTol", 1e-9);
            tc.verifyEqual(b.Position(3), z0 - 0.5*g*t^2, "RelTol", 0.02);
            % No lateral drift for a pure vertical drop.
            tc.verifyEqual(b.Position(1:2), [0 0], "AbsTol", 1e-6);
        end

        function staticBodyDoesNotMove(tc)
            b = tc.spawnBody([1 2 5], "static");
            sim = phx.Simulation(b);
            tc.addTeardown(@() delete(sim));
            sim.step(1, 200);
            tc.verifyEqual(b.Position, [1 2 5], "AbsTol", 1e-9);
        end

        function timeAccumulatesAcrossSteps(tc)
            b = tc.spawnBody([0 0 20], "dynamic");
            sim = phx.Simulation(b);
            tc.addTeardown(@() delete(sim));
            sim.step(0.1, 10);
            sim.step(0.1, 10);
            tc.verifyEqual(sim.Time, 0.2, "AbsTol", 1e-9);
        end

        function gravityChangeAppliesToExistingWorld(tc)
            % Regression: assigning Gravity after construction must reach
            % the engine, not just the MATLAB property.
            g = 9.81;
            b = tc.spawnBody([0 0 50], "dynamic");
            sim = phx.Simulation(b);
            tc.addTeardown(@() delete(sim));

            sim.Gravity = [0 0 0];
            sim.step(0.5, 100);
            tc.verifyEqual(b.LinearVelocity, [0 0 0], "AbsTol", 1e-9);

            sim.Gravity = [0 0 -g];
            sim.step(0.5, 100);
            tc.verifyEqual(b.LinearVelocity(3), -g*0.5, "RelTol", 0.02);
        end

        function simulationIsDeterministic(tc)
            p1 = tc.runDrop();
            p2 = tc.runDrop();
            tc.verifyEqual(p1, p2, "AbsTol", 1e-12);
        end
    end

    methods (Test, TestTags = {'Engine', 'Graphics'})
        function meshPhysicsIsDrawPathIndependent(tc)
            % Regression guard: mesh collision hulls (STL and friends with
            % the convex envelope) are built from the vertices of the drawn
            % primitive, so every draw path - headless ([] parent), phx axes
            % (world primitives) and plain axes (patches) - must carry
            % identical geometry and produce bit-for-bit identical physics.
            % The STL coordinates are chosen not to be representable in
            % float32, which is what used to tell the draw paths apart.
            folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            stl = fullfile(folder, "tumbler.stl");
            tSimulation.writeTumblerSTL(stl);

            pHeadless = tc.tumbleMesh([], stl);

            f = figure("Visible", "off");
            tc.addTeardown(@() close(f));
            [~, axViewer] = phx.extra.Viewer(axes(f));
            pViewer = tc.tumbleMesh(axViewer, stl);

            f2 = figure("Visible", "off");
            tc.addTeardown(@() close(f2));
            pPlain = tc.tumbleMesh(axes(f2), stl);

            tc.verifyEqual(pViewer, pHeadless, ...
                "A phx-axes run differs from the headless run.");
            tc.verifyEqual(pPlain, pHeadless, ...
                "A plain-axes run differs from the headless run.");
        end
    end

    methods (Access = private)
        function p = tumbleMesh(~, ax, stl)
            % An irregular tetrahedron tumbling down a tilted plate; any
            % difference in the collision hull shows up in the final pose
            ground = phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], ...
                "EulerAngles", [0 0.3 0], "Shape", {"Box", "Size", [20 4 1]}, ...
                "Friction", 0.3);
            body = phx.Body(ax, "Position", [-3 0 2], "Friction", 0.3, ...
                "Shape", {"STL", "Source", stl, "Envelope", "convex"});
            sim = phx.Simulation([ground body]);
            sim.step(3, 600, -1);
            p = [body.Position body.Quaternion];
            delete(sim);
        end
        function b = spawnBody(tc, position, type)
            f = figure("Visible", "off");
            tc.addTeardown(@() close(f));
            b = phx.Body(axes(f), "Position", position, "Type", type);
        end

        function p = runDrop(tc) %#ok<MANU>
            f = figure("Visible", "off");
            b = phx.Body(axes(f), "Position", [0.1 0.2 8], "AngularVelocity", [1 2 3]);
            sim = phx.Simulation(b);
            sim.step(0.5, 500);
            p = b.Transform;
            delete(sim);
            close(f);
        end
    end

    methods (Static, Access = private)
        function writeTumblerSTL(file)
            % An irregular tetrahedron whose coordinates are exact in
            % float64 but not in float32
            A = "-0.3 -0.2 -0.1"; B = "0.3 -0.3 0.5"; C = "0.0 0.4 -0.2"; D = "0.5 0.2 0.1";
            lines = ["solid tumbler"];
            for tri = {{A B C}, {A B D}, {A C D}, {B C D}}
                lines(end + 1) = "  facet normal 0 0 1"; %#ok<AGROW> four facets
                lines(end + 1) = "    outer loop"; %#ok<AGROW>
                lines(end + 1) = "      vertex " + tri{1}{1}; %#ok<AGROW>
                lines(end + 1) = "      vertex " + tri{1}{2}; %#ok<AGROW>
                lines(end + 1) = "      vertex " + tri{1}{3}; %#ok<AGROW>
                lines(end + 1) = "    endloop"; %#ok<AGROW>
                lines(end + 1) = "  endfacet"; %#ok<AGROW>
            end
            lines(end + 1) = "endsolid tumbler";
            fid = fopen(file, "w");
            fwrite(fid, strjoin(lines, newline));
            fclose(fid);
        end
    end

end
