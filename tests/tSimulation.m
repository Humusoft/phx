classdef tSimulation < matlab.unittest.TestCase
%tSimulation Integration tests that exercise the physics engine (MEX).
%
%   These verify end-to-end behavior: a body really falls, a static body
%   really stays put, and repeated runs are bit-for-bit reproducible (the
%   determinism guarantee demonstrated by demo_determinism). They depend on
%   the engine binary and on graphics, so every test carries the "Engine"
%   tag and can be excluded on machines without the MEX.
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

        function simulationIsDeterministic(tc)
            p1 = tc.runDrop();
            p2 = tc.runDrop();
            tc.verifyEqual(p1, p2, "AbsTol", 1e-12);
        end
    end

    methods (Access = private)
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

end
