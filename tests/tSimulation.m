classdef tSimulation < PhxTestCase
%tSimulation Integration tests that exercise the physics engine (MEX).
%
%   These verify end-to-end behavior: a body really falls, a static body
%   really stays put, and repeated runs are bit-for-bit reproducible (the
%   determinism guarantee demonstrated by phxex_determinism). The bodies are
%   built headless ([] axes), so only the engine is needed and every test
%   carries the "Engine" tag; the draw-path independence test is the one that
%   compares the three draw paths against each other and therefore
%   additionally carries the "Graphics" tag.
%
%   See also phx.Simulation, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        % An irregular tetrahedron whose coordinates are exact in float64 but
        % not in float32, which is what used to tell the draw paths apart
        TumblerVertices = [-0.3 -0.2 -0.1; 0.3 -0.3 0.5; 0.0 0.4 -0.2; 0.5 0.2 0.1]
        TumblerFaces = [1 2 3; 1 2 4; 1 3 4; 2 3 4]
    end

    methods (TestClassSetup)
        function engineIsPresent(tc)
            tc.requireEngine;
        end
    end

    methods (Test, TestTags = {'Engine'})
        function freeFallMatchesAnalyticDrop(tc)
            g = 9.81;
            t = 1.0;
            z0 = 50;                       % start high, no ground to hit
            b = tc.spawnBody([0 0 z0]);
            sim = phx.Simulation(b, "Gravity", [0 0 -g]);
            tc.addTeardown(@() delete(sim));

            sim.step(t, 1000);

            tc.verifyEqual(sim.Time, t, "AbsTol", 1e-9);
            tc.verifyEqual(b.Position(3), z0 - 0.5*g*t^2, "RelTol", 0.02);
            % No lateral drift for a pure vertical drop.
            tc.verifyEqual(b.Position(1:2), [0 0], "AbsTol", 1e-6);
        end

        function staticBodyDoesNotMove(tc)
            b = tc.spawnBody([1 2 5], "Type", "static");
            sim = phx.Simulation(b);
            tc.addTeardown(@() delete(sim));
            sim.step(1, 200);
            tc.verifyEqual(b.Position, [1 2 5], "AbsTol", 1e-9);
        end

        function timeAccumulatesAcrossSteps(tc)
            b = tc.spawnBody([0 0 20]);
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
            b = tc.spawnBody([0 0 50]);
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

        function rollingResistanceAndSpinRadiusReachTheEngine(tc)
            % A ball rolling and spinning on a floor slows by its rolling
            % resistance b (b g / (1.4 R)) and its spin radius r
            % (mu r m g / I), set at construction; set to 0 while running,
            % they stop acting
            g = 9.81;
            R = 0.1;
            floor = tc.spawnBody([0 0 -0.5], "Type", "static", ...
                "Shape", {"Box", "Size", [20 20 1]}, "Friction", 1);
            ball = tc.spawnBody([0 0 R], "Shape", {"Sphere", "Radius", R}, ...
                "Mass", 1, "Inertia", 0.4*R^2*[1 1 1], "Friction", 0.25, ...   % 0.5 with the floor's 1
                "RollingResistance", 0.002, "SpinRadius", 0.01, ...
                "LinearVelocity", [1 0 0], "AngularVelocity", [0 1/R 5]);
            sim = phx.Simulation([floor ball], "Gravity", [0 0 -g]);
            tc.addTeardown(@() delete(sim));
            sim.step(0.05, 10, -1);
            v1 = ball.LinearVelocity;
            w1 = ball.AngularVelocity;
            sim.step(0.2, 40, -1);
            v2 = ball.LinearVelocity;
            w2 = ball.AngularVelocity;
            tc.verifyEqual((v1(1) - v2(1))/0.2, 0.002*g/(1.4*R), "RelTol", 0.02, "Rolling");
            tc.verifyEqual((w1(3) - w2(3))/0.2, 0.5*0.01*g/(0.4*R^2), "RelTol", 0.02, "Spinning");

            ball.RollingResistance = 0;
            ball.SpinRadius = 0;
            sim.step(0.05, 10, -1);
            v3 = ball.LinearVelocity;
            w3 = ball.AngularVelocity;
            sim.step(0.2, 40, -1);
            tc.verifyEqual(ball.LinearVelocity(1), v3(1), "RelTol", 1e-3, "Rolling without resistance");
            tc.verifyEqual(ball.AngularVelocity(3), w3(3), "RelTol", 1e-3, "Spinning without spin radius");
        end
    end

    methods (Test, TestTags = {'Engine', 'Graphics'})
        function meshPhysicsIsDrawPathIndependent(tc)
            % Regression guard: mesh collision hulls (STL and friends with
            % the convex envelope) are built from the vertices of the drawn
            % primitive, so every draw path - headless ([] parent), phx axes
            % (world primitives) and plain axes (patches) - must carry
            % identical geometry and produce bit-for-bit identical physics.
            stl = tc.writeSTL("tumbler.stl", tc.TumblerVertices, tc.TumblerFaces);

            pHeadless = tc.tumbleMesh([], stl);

            [~, axViewer] = phx.extra.Viewer(tc.prepareAxes);
            pViewer = tc.tumbleMesh(axViewer, stl);

            pPlain = tc.tumbleMesh(tc.prepareAxes, stl);

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
                "Shape", {"Mesh", "Source", stl, "Envelope", "convex"});
            sim = phx.Simulation([ground body]);
            sim.step(3, 600, -1);
            p = [body.Position body.Quaternion];
            delete(sim);
        end

        function p = runDrop(tc)
            b = tc.spawnBody([0.1 0.2 8], "AngularVelocity", [1 2 3]);
            sim = phx.Simulation(b);
            sim.step(0.5, 500);
            p = b.Transform;
            delete(sim);
            delete(b);
        end
    end

end
