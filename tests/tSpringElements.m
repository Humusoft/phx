classdef tSpringElements < PhxTestCase
%tSpringElements Tests for phx.Spring, phx.Rope and phx.TorsionSpring.
%
%   The engine solves all three elements as constraints. The configuration
%   tests check what the elements report before a simulation exists and
%   the documented errors; they need neither graphics nor the engine. The
%   physics tests compare a static equilibrium or an oscillation period
%   with its closed form and carry the "Engine" tag.
%
%   See also phx.Spring, phx.Rope, phx.TorsionSpring

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        G = 9.81
        Dt = 0.005
    end

    % --- Configuration (no engine) ----------------------------------------
    methods (Test)
        function springReportsGeometryBeforeSimulation(tc)
            a = tc.spawnBody([0 0 0], "Type", "static");
            b = tc.spawnBody([0 3 4]);
            s = phx.Spring(a, b, "FreeLength", 2, "Stiffness", 100);
            tc.verifyEqual(s.Length, 5, "AbsTol", 1e-12);
            tc.verifyEqual(s.Force, [0 0 0]);
            tc.verifyEqual(s.Energy, 0.5*100*(5 - 2)^2, "RelTol", 1e-12);
        end

        function springRejectsNegativeParameters(tc)
            a = tc.spawnBody([0 0 0]);
            b = tc.spawnBody([0 0 2]);
            tc.verifyError(@() phx.Spring(a, b, "Stiffness", -1), "MATLAB:validators:mustBeNonnegative");
            tc.verifyError(@() phx.Spring(a, b, "Inertance", -1), "MATLAB:validators:mustBeNonnegative");
            tc.verifyEmpty(a.Children, "A failed construction must leave nothing behind.");
        end

        function ropeNeedsTwoBodies(tc)
            a = tc.spawnBody([0 0 0]);
            tc.verifyError(@() phx.Rope(a), "phx:Rope:tooFewBodies");
        end

        function ropeStartsAtItsRoutedLength(tc)
            a = tc.spawnBody([0 0 0], "Type", "static");
            p = tc.spawnBody([4 0 0], "Type", "static");
            b = tc.spawnBody([4 0 -3]);
            r = phx.Rope([a p b]);
            tc.verifyEqual(r.Length, 7, "AbsTol", 1e-12);
            tc.verifyEqual(r.FreeLength, 7, "AbsTol", 1e-12, "InitialLength defaults to the routed length.");
            r.Displacement = -1;
            tc.verifyEqual(r.FreeLength, 6, "AbsTol", 1e-12);
        end

        function torsionSpringAngleBeforeSimulation(tc)
            a = tc.spawnBody([0 0 0], "Type", "static");
            b = tc.spawnBody([0 0 0], "EulerAngles", [0 0 0.4]);
            t = phx.TorsionSpring(a, b, "Stiffness", 2, "FreeAngle", 0.1);
            tc.verifyEqual(t.Angle, 0.4, "AbsTol", 1e-9);
            tc.verifyEqual(t.Torque, [0 0 0]);
            tc.verifyEqual(t.Energy, 0.5*2*(0.4 - 0.1)^2, "RelTol", 1e-9);
        end

        function colormapDrawsTheLineAndKeepsColor(tc)
            % A colormap draws the line (zero force picks its first row),
            % Color keeps its own value meanwhile and is drawn again once
            % the colormap is emptied
            a = tc.spawnBody([0 0 0], "Type", "static");
            b = tc.spawnBody([0 0 -2]);
            map = [0 0 1; 1 0 0];
            elements = {phx.Spring(a, b, "Color", [0 1 0], "Colormap", map), ...
                phx.Rope([a b], "Color", [0 1 0], "Colormap", map)};
            for k = 1:numel(elements)
                e = elements{k};
                tc.verifyEqual(tc.lineColor(e), [0 0 1], class(e));
                tc.verifyEqual(e.Color, [0 1 0], class(e));
                e.Color = [1 1 0];
                tc.verifyEqual(e.Color, [1 1 0], class(e));
                e.Colormap = [];
                tc.verifyEqual(tc.lineColor(e), [1 1 0], class(e));
            end
        end

        function colormapIsAMatrixNotAName(tc)
            a = tc.spawnBody([0 0 0]);
            b = tc.spawnBody([0 0 2]);
            tc.verifyError(@() phx.Spring(a, b, "Colormap", "jet"), ?MException);
            tc.verifyError(@() phx.Rope([a b], "Colormap", "jet"), ?MException);
            tc.verifyEmpty(a.Children, "A failed construction must leave nothing behind.");
        end
    end

    % --- Physics (engine) ---------------------------------------------------
    methods (Test, TestTags = {'Engine'})
        function springHoldsAHangingMass(tc)
            % A mass hanging on a damped spring settles at the extension
            % m*g/k, and the spring pulls the anchor down with the weight
            tc.requireEngine;
            m = 2; k = 400; L0 = 1;
            [a, b] = tc.anchorAndMass([0 0 -2], m);
            s = phx.Spring(a, b, "FreeLength", L0, "Stiffness", k, "Damping", 40);
            sim = tc.simulate({a, b});
            sim.step(5, 5/tc.Dt, -1);
            tc.verifyEqual(s.Length - L0, m*tc.G/k, "RelTol", 0.01);
            tc.verifyEqual(s.Force, [0 0 -m*tc.G], "AbsTol", 0.01*m*tc.G);
        end

        function springPeriodFollowsMassAndInertance(tc)
            % Undamped oscillation at T = 2*pi*sqrt((m + b)/k); the inerter
            % adds its inertance b to the moving mass
            tc.requireEngine;
            m = 1; k = 100;
            for b = [0 1]
                [a, body] = tc.anchorAndMass([0 0 -1.5], m);
                phx.Spring(a, body, "FreeLength", 1, "Stiffness", k, "Inertance", b);
                sim = tc.simulate({a, body}, [0 0 0]);
                T = tc.period(sim, @() body.Position(3) + 1, 2);
                tc.verifyEqual(T, 2*pi*sqrt((m + b)/k), "RelTol", 0.01, ...
                    sprintf("Period with inertance %g.", b));
                delete(sim);
            end
        end

        function stiffnessChangeReachesARunningSimulation(tc)
            tc.requireEngine;
            m = 1;
            [a, b] = tc.anchorAndMass([0 0 -2], m);
            s = phx.Spring(a, b, "FreeLength", 1, "Stiffness", 200, "Damping", 30);
            sim = tc.simulate({a, b});
            sim.step(3, 3/tc.Dt, -1);
            s.Stiffness = 100;
            sim.step(4, 4/tc.Dt, -1);
            tc.verifyEqual(s.Length - 1, m*tc.G/100, "RelTol", 0.01);
        end

        function colormapFollowsTheForceAfterSaveAndLoad(tc)
            % A saved and reloaded spring keeps its colormap and Color, and
            % the colormap keeps coloring it by the force
            tc.requireEngine;
            [a, b] = tc.anchorAndMass([0 0 -2], 1);
            map = gray(11);
            phx.Spring(a, b, "FreeLength", 1, "Stiffness", 100, "Color", [0 1 0], ...
                "Colormap", map, "ColorRange", [0 20]);
            sim = tc.simulate({a, b});
            sim.step(0.1, 20, -1);
            delete(sim);

            file = fullfile(tc.tempFolder, "spring.mat");
            save(file, "a", "b");
            scene = load(file);
            s = scene.a.Children{1};
            tc.verifyEqual(s.Colormap, map);
            tc.verifyEqual(s.Color, [0 1 0]);

            sim = tc.simulate({scene.a, scene.b});
            sim.step(0.1, 20, 0);                  % redraw at the end of the step
            row = phx.Spring.x2id(norm(s.Force), s.ColorRange, [1 size(map, 1)]);
            tc.verifyEqual(tc.lineColor(s), map(row, :), "AbsTol", 1/255);
            tc.verifyGreaterThan(row, 1, "The spring must carry a force.");
        end

        function ropeCarriesTheWeightOverAPulley(tc)
            % A rope from a fixed point over a pulley holds a hanging mass:
            % its tension is the weight, its stretch m*g/k
            tc.requireEngine;
            m = 3; k = 2000;
            a = tc.spawnBody([0 0 0], "Type", "static", "Shape", {"Sphere", "Radius", 0.05});
            p = tc.spawnBody([2 0 0], "Type", "static", "Shape", {"Sphere", "Radius", 0.05});
            b = tc.spawnBody([2 0 -1], "Mass", m, "Shape", {"Sphere", "Radius", 0.05});
            r = phx.Rope([a p b], "Stiffness", k, "Damping", 100);
            sim = tc.simulate({a, p, b});
            sim.step(5, 5/tc.Dt, -1);
            tc.verifyEqual(r.Force, m*tc.G, "RelTol", 0.01);
            tc.verifyEqual(r.Length - r.FreeLength, m*tc.G/k, "RelTol", 0.02);
        end

        function torsionSpringOscillatesAndWinds(tc)
            % A body spun about the spring axis swings at T = 2*pi*sqrt(I/k);
            % an amplitude above pi is followed past the half turn
            tc.requireEngine;
            I = 0.1; k = 0.4;
            a = tc.spawnBody([0 0 0], "Type", "static");
            b = tc.spawnBody([0 0 2], "Mass", 1, "Inertia", [I I I]);
            t = phx.TorsionSpring(a, b, "Stiffness", k);
            sim = tc.simulate({a, b}, [0 0 0]);
            w0 = 8;                                % amplitude w0*sqrt(I/k) = 4 rad
            b.AngularVelocity = [0 0 w0];
            peak = 0;
            for i = 1:round(1.2/tc.Dt)             % past the quarter period 0.79 s
                sim.step(tc.Dt, 1, -1);
                peak = max(peak, t.Angle);
            end
            tc.verifyEqual(peak, w0*sqrt(I/k), "RelTol", 0.01, "The angle must not wrap at pi.");
            T = tc.period(sim, @() t.Angle, 2);
            tc.verifyEqual(T, 2*pi*sqrt(I/k), "RelTol", 0.01);
            tc.verifyEqual(t.Energy, 0.5*k*t.Angle^2, "RelTol", 1e-9);
        end
    end

    methods (Access = private)
        function rgb = lineColor(~, element)
            % Color the line of a spring or a rope is drawn in
            h = findobj(element.Graphics.Children, "-isa", "matlab.graphics.primitive.world.LineStrip");
            rgb = double(h.ColorData(1:3)')/255;
        end

        function [a, b] = anchorAndMass(tc, position, m)
            % Static anchor at the origin and a small sphere of mass m
            a = tc.spawnBody([0 0 0], "Type", "static", "Shape", {"Sphere", "Radius", 0.05});
            b = tc.spawnBody(position, "Mass", m, "Shape", {"Sphere", "Radius", 0.05});
        end

        function sim = simulate(tc, objects, gravity)
            arguments
                tc
                objects
                gravity (1, 3) double = [0 0 -tc.G]
            end
            sim = phx.Simulation(objects, "Gravity", gravity);
            tc.addTeardown(@() tc.deleteIfValid(sim));
        end

        function T = period(tc, sim, signal, n)
            % Mean period over n full cycles, from the upward zero crossings
            % of signal, interpolated within the step
            times = [];
            last = signal();
            while numel(times) < n + 1 && sim.Time < 30
                sim.step(tc.Dt, 1, -1);
                now = signal();
                if last < 0 && now >= 0
                    times(end + 1) = sim.Time - tc.Dt*now/(now - last); %#ok<AGROW> few crossings
                end
                last = now;
            end
            T = mean(diff(times));
        end
    end

end
