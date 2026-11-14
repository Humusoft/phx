classdef tParticles < PhxTestCase
%tParticles Tests for phx.Particles.
%
%   The bookkeeping tests add, remove and set particles before a simulation
%   exists and check the documented errors; they need neither graphics nor
%   the engine. The simulation tests ("Engine" tag) check that a running
%   group reads and writes its particles through the engine, that a force
%   changes the velocity of its particle by F*dt/m, that particles come to
%   rest on a floor, and that the state survives the end of the simulation.
%
%   See also phx.Particles

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        R = 0.05     % particle radius
        Dt = 0.005
    end

    % --- Bookkeeping (no engine) ------------------------------------------
    methods (Test)
        function groupReportsItsParticles(tc)
            P = [0 0 1; 1 0 1; 0 1 2];
            p = phx.Particles([], "Positions", P, "Radius", tc.R, "Density", 2000);
            tc.verifyEqual(p.Count, 3);
            tc.verifyEqual(p.Positions, P);
            tc.verifyEqual(p.Velocities, zeros(3));
            tc.verifyEqual(p.ParticleMass, 2000*4/3*pi*tc.R^3, "RelTol", 1e-12);
            tc.verifyEqual(p.Mass, 3*p.ParticleMass, "RelTol", 1e-12);
            tc.verifyEqual(p.boundingBox, [min(P) - tc.R; max(P) + tc.R], "AbsTol", 1e-12);
        end

        function massesSetTheDensity(tc)
            % Mass and ParticleMass are views of Density; given to the
            % constructor before Positions and Radius they still count them
            p = phx.Particles([], "Mass", 6, "Positions", [0 0 1; 1 0 1; 2 0 1], "Radius", tc.R);
            tc.verifyEqual(p.Mass, 6, "RelTol", 1e-12);
            tc.verifyEqual(p.Density, 2/(4/3*pi*tc.R^3), "RelTol", 1e-12);
            p.ParticleMass = 0.5;
            tc.verifyEqual(p.Mass, 1.5, "RelTol", 1e-12);
            p.add([3 0 1]);
            tc.verifyEqual(p.Mass, 2, "RelTol", 1e-12, "Density stays as particles come.");
            tc.verifyError(@() set(phx.Particles([]), "Mass", 1), "phx:Particles:noParticles");
        end

        function emptyGroupHasNoBoundingBox(tc)
            p = phx.Particles([]);
            tc.verifyEqual(p.Count, 0);
            tc.verifyEqual(p.boundingBox, [Inf Inf Inf; -Inf -Inf -Inf]);
        end

        function addAndRemoveKeepTheOrder(tc)
            p = phx.Particles([], "Positions", [0 0 1; 1 0 1]);
            p.add([2 0 1; 3 0 1], [0 0 1; 0 0 2]);
            tc.verifyEqual(p.Positions(:, 1)', 0:3);
            tc.verifyEqual(p.Velocities(:, 3)', [0 0 1 2]);
            p.remove(2);
            tc.verifyEqual(p.Positions(:, 1)', [0 2 3]);
            p.remove([true false true]);
            tc.verifyEqual(p.Positions, [2 0 1]);
        end

        function invalidInputsAreRejected(tc)
            p = phx.Particles([], "Positions", [0 0 1; 1 0 1]);
            tc.verifyError(@() p.add([0 0 0], [0 0 0; 1 1 1]), "phx:Particles:sizeMismatch");
            tc.verifyError(@() p.remove(3), "phx:Particles:badIndex");
            tc.verifyError(@() p.remove([true false true]), "phx:Particles:sizeMismatch");
            tc.verifyError(@() set(p, "Velocities", [0 0 0]), "phx:Particles:sizeMismatch");
            tc.verifyError(@() p.applyForce([0 0 1]), "phx:Particles:notSimulated");
            tc.verifyError(@() phx.Particles([], "Positions", zeros(phx.Particles.MaxCount + 1, 3)), ...
                "phx:Particles:tooMany");
        end
    end

    % --- Simulation (engine) ------------------------------------------------
    methods (Test, TestTags = {'Engine'})
        function runningGroupReadsAndWritesTheEngine(tc)
            tc.requireEngine;
            p = phx.Particles([], "Positions", [0 0 1; 1 0 1], "Radius", tc.R);
            sim = tc.simulate({p}, [0 0 0]);
            p.Positions = [0 0 2; 1 0 2];
            p.Velocities = [1 0 0; 0 0 0];
            sim.step(0.1, 0.1/tc.Dt, -1);
            tc.verifyEqual(p.Positions, [0.1 0 2; 1 0 2], "AbsTol", 1e-6);
            p.add([2 0 2]);
            tc.verifyEqual(p.Count, 3);
            p.remove(1);
            tc.verifyEqual(p.Positions(:, 1)', [1 2], "AbsTol", 1e-6);
        end

        function forceChangesTheVelocityOfItsParticle(tc)
            tc.requireEngine;
            p = phx.Particles([], "Positions", [0 0 1; 1 0 1], "Radius", tc.R);
            sim = tc.simulate({p}, [0 0 0]);
            F = [0.5 0 0; 0 0 0];
            p.applyForce(F);
            sim.step(tc.Dt, 1, -1);
            tc.verifyEqual(p.Velocities, F*tc.Dt/p.ParticleMass, "AbsTol", 1e-6);
        end

        function particlesComeToRestOnTheFloor(tc)
            tc.requireEngine;
            floor = tc.spawnBody([0 0 -0.5], "Type", "static", "Shape", {"Box", "Size", [4 4 1]});
            [x, y] = meshgrid(-0.3:0.15:0.3);
            p = phx.Particles([], "Positions", [x(:) y(:) 0.5 + 0*x(:)], "Radius", tc.R);
            sim = tc.simulate({floor, p});
            sim.step(1.5, 1.5/tc.Dt, -1);
            tc.verifyEqual(p.Positions(:, 3), tc.R + 0*x(:), "AbsTol", 0.1*tc.R);
            tc.verifyLessThan(max(vecnorm(p.Velocities, 2, 2)), 0.01);
        end

        function stateSurvivesTheSimulation(tc)
            tc.requireEngine;
            p = phx.Particles([], "Positions", [0 0 1], "Radius", tc.R);
            sim = tc.simulate({p}, [0 0 0]);
            p.Velocities = [0 0 -1];
            sim.step(0.2, 0.2/tc.Dt, -1);
            delete(sim);
            tc.verifyEqual(p.Positions, [0 0 0.8], "AbsTol", 1e-6);
            tc.verifyEqual(p.Velocities, [0 0 -1], "AbsTol", 1e-6);
        end
    end

    methods (Access = private)
        function sim = simulate(tc, objects, gravity)
            arguments
                tc
                objects
                gravity (1, 3) double = [0 0 -9.81]
            end
            sim = phx.Simulation(objects, "Gravity", gravity);
            tc.addTeardown(@() tc.deleteIfValid(sim));
        end
    end

end
