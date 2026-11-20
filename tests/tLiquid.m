classdef tLiquid < PhxTestCase
%tLiquid Tests for phx.Liquid and the Shape that fills a particle group.
%
%   The bookkeeping tests need neither graphics nor the engine: the
%   defaults of a liquid, the mass of its particles, filling a Shape (the
%   lattice of the Spacing, the pose, the order of the constructor's
%   arguments, Positions replacing the shape, the shapes that cannot fill)
%   and the errors. The simulation tests ("Engine" tag) check that the
%   engine takes the mass PHX reports, that a lattice at rest reads the
%   Density and a changed Density reaches the engine, that a column in a
%   tank keeps its level, that the Shape is done once the simulation holds
%   the group, and that the state survives the end of the simulation. The
%   physics of the liquid itself is the engine's contract (tEngineProtocol,
%   fluid* tests).
%
%   See also phx.Liquid, phx.base.ParticleGroup, tParticles

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        D = 0.02     % spacing
        Dt = 2.5e-4
    end

    properties (TestParameter)
        % Shapes that fill, with their volume; at a spacing of a twentieth
        % of their size the particles fill it within 3 %
        Filled = struct( ...
            "Box", {{{"Box", "Size", [0.2 0.1 0.3]}, 0.2*0.1*0.3}}, ...
            "Sphere", {{{"Sphere", "Diameter", 0.2}, 4/3*pi*0.1^3}}, ...
            "Cylinder", {{{"Cylinder", "Diameter", 0.2, "Height", 0.3}, pi*0.1^2*0.3}}, ...
            "Cone", {{{"Cone", "Diameter", 0.2, "Height", 0.3}, pi*0.1^2*0.3/3}}, ...
            "Capsule", {{{"Capsule", "Diameter", 0.1, "Height", 0.2}, pi*0.05^2*0.2 + 4/3*pi*0.05^3}}, ...
            "Tube", {{{"Tube", "Diameter", 0.3, "WallThickness", 0.06, "Height", 0.2}, pi*(0.15^2 - 0.09^2)*0.2}})
    end

    % --- Bookkeeping (no engine) ------------------------------------------
    methods (Test)
        function liquidHasItsDefaults(tc)
            % A liquid slides along walls without bouncing; the mass of a
            % particle is that of the lattice that reads the Density
            L = phx.Liquid([], "Positions", [0 0 1; 1 0 1]);
            tc.verifyEqual([L.Friction L.Restitution], [0 0]);
            tc.verifyEqual([L.Spacing L.Density L.SoundSpeed L.Viscosity], [0.02 1000 20 0.1]);
            tc.verifyEqual(L.ParticleMass, 1000*0.02^3*32768*pi/103950, "RelTol", 1e-12);
            tc.verifyEqual(L.ParticleMass/(1000*0.02^3), 0.990, "AbsTol", 5e-4);
            tc.verifyEqual(L.Mass, 2*L.ParticleMass, "RelTol", 1e-12);
            tc.verifyEqual(L.boundingBox, [0 0 1; 1 0 1] + [-0.01; 0.01], "AbsTol", 1e-12);
            tc.verifySize(L.Densities, [0 1], "No densities outside a simulation.");
            tc.verifySize(L.Pressures, [0 1]);
        end

        function liquidHasNoParticleLimit(tc)
            % The limit of phx.Particles comes from contacts among the
            % particles, which a liquid has not
            L = phx.Liquid([], "Positions", zeros(phx.Particles.MaxCount + 1, 3));
            tc.verifyEqual(L.Count, phx.Particles.MaxCount + 1);
        end

        function shapeFillsWithTheLatticeOfTheSpacing(tc, Filled)
            L = phx.Liquid([], "Shape", Filled{1}, "Spacing", 0.01);
            tc.verifyEqual(L.Count*0.01^3, Filled{2}, "RelTol", 0.03);
            tc.verifyCubicLattice(L.Positions, 0.01, "A cubic lattice of the Spacing.");
        end

        function boxFillsExactly(tc)
            % A box of whole spacings holds them all, d/2 inside its faces
            L = phx.Liquid([], "Shape", {"Box", "Size", [0.2 0.1 0.3]}, "Spacing", tc.D);
            tc.verifyEqual(L.Count, 10*5*15);
            tc.verifyEqual(L.boundingBox, [-0.1 -0.05 -0.15; 0.1 0.05 0.15], "AbsTol", 1e-12);
        end

        function shapeIsPlacedAtItsPose(tc)
            % Position and Orientation place the shape, whatever the order
            % of the constructor's arguments
            box = {"Box", "Size", [0.2 0.1 0.3]};
            A = phx.Liquid([], "Shape", box, "Spacing", 0.01, "Position", [1 2 3], "EulerAngles", [0 0 pi/2]);
            B = phx.Liquid([], "EulerAngles", [0 0 pi/2], "Position", [1 2 3], "Spacing", 0.01, "Shape", box);
            tc.verifyEqual(sortrows(A.Positions), sortrows(B.Positions), "AbsTol", 1e-12);
            tc.verifyEqual(mean(A.Positions), [1 2 3], "AbsTol", 1e-12);
            tc.verifyEqual(max(A.Positions) - min(A.Positions), [0.09 0.19 0.29], "AbsTol", 1e-12, "Turned a quarter about z.");
        end

        function shapeIsFilledAgainUntilTheSimulation(tc)
            % A new Position or Spacing fills the shape again; Positions
            % given replace it, and a later Position moves nothing
            L = phx.Liquid([], "Shape", {"Box", "Size", [0.2 0.2 0.2]}, "Spacing", tc.D);
            tc.verifyEqual(L.Count, 1000);
            L.Spacing = 0.04;
            tc.verifyEqual(L.Count, 125);
            L.Position = [0 0 1];
            tc.verifyEqual(mean(L.Positions), [0 0 1], "AbsTol", 1e-12);
            L.Positions = [0 0 0; 1 0 0];
            L.Position = [5 5 5];
            tc.verifyEqual(L.Positions, [0 0 0; 1 0 0]);
        end

        function shapeErrors(tc)
            tc.verifyError(@() phx.Liquid([], "Shape", {"Box"}, "Positions", [0 0 0]), "phx:Liquid:shapeAndPositions");
            tc.verifyError(@() phx.Liquid([], "Shape", {"Revolution"}), "phx:Liquid:badShape");
            tc.verifyError(@() phx.Liquid([], "Shape", 42), "phx:Liquid:badShape");
            L = phx.Liquid([], "Positions", [0 0 1]);
            tc.verifyError(@() L.applyForce([0 0 1]), "phx:Liquid:notSimulated");
            tc.verifyError(@() set(L, "Velocities", [0 0 0; 0 0 0]), "phx:Liquid:sizeMismatch");
        end
    end

    % --- Simulation (engine) ------------------------------------------------
    methods (Test, TestTags = {'Engine'})
        function engineTakesTheMassOfTheParticles(tc)
            % A lone particle has no neighbours and no pressure: a force
            % changes its velocity by F*dt/ParticleMass
            tc.requireLiquids;
            L = phx.Liquid([], "Positions", [0 0 1], "Spacing", tc.D);
            sim = tc.simulate({L}, [0 0 0]);
            L.applyForce([1 0 0]);
            sim.step(tc.Dt, 1, -1);
            tc.verifyEqual(L.Velocities, [1 0 0]*tc.Dt/L.ParticleMass, "RelTol", 1e-5);
        end

        function latticeReadsTheDensity(tc)
            % The middle of a lattice at rest reads the Density at no
            % pressure; a new Density reaches the running liquid
            tc.requireLiquids;
            L = phx.Liquid([], "Shape", {"Box", "Size", 5*tc.D*[1 1 1]}, "Spacing", tc.D);
            tc.simulate({L}, [0 0 0]);
            centre = find(all(abs(L.Positions) < 1e-9, 2));
            rho = L.Densities;
            tc.verifySize(rho, [125 1]);
            tc.verifyEqual(rho(centre), 1000, "RelTol", 1e-6);
            % The pressure of that density error: rho0*c^2*1e-6
            tc.verifyEqual(L.Pressures(centre), 0, "AbsTol", 1000*L.SoundSpeed^2*1e-6);
            L.Density = 800;
            tc.verifyEqual(L.Densities(centre), 800, "RelTol", 1e-6);
        end

        function columnKeepsItsLevelInATank(tc)
            % A column at rest in an arena keeps its level within half a
            % spacing and comes to rest
            tc.requireLiquids;
            parts = phx.assembly.arena([], "Size", [0.2 0.2 0.3], "Thickness", 0.05, "Friction", 0);
            L = phx.Liquid([], "Shape", {"Box", "Size", [0.2 0.2 0.16]}, "Position", [0 0 0.08], "Spacing", tc.D);
            sim = tc.simulate([{parts.floor}, num2cell(parts.walls), {L}]);
            sim.step(1, 1/tc.Dt, -1);
            top = sort(L.Positions(:, 3), "descend");
            tc.verifyEqual(mean(top(1:25)) + tc.D/2, 0.16, "AbsTol", tc.D/2);
            tc.verifyLessThan(max(L.Densities), 1000*1.02);
            tc.verifyLessThan(max(vecnorm(L.Velocities, 2, 2)), 0.2);
        end

        function shapeIsDoneOnceSimulated(tc)
            tc.requireLiquids;
            L = phx.Liquid([], "Shape", {"Box", "Size", 3*tc.D*[1 1 1]}, "Position", [0 0 1], "Spacing", tc.D);
            sim = tc.simulate({L}, [0 0 0]);
            tc.verifyError(@() set(L, "Shape", {"Sphere"}), "phx:Liquid:running");
            sim.step(tc.Dt, 1, -1);
            delete(sim);
            P = L.Positions;
            L.Position = [5 5 5];
            tc.verifyEqual(L.Positions, P, "The shape no longer moves the particles.");
        end

        function stateSurvivesTheSimulation(tc)
            tc.requireLiquids;
            L = phx.Liquid([], "Positions", [0 0 1], "Spacing", tc.D);
            sim = tc.simulate({L}, [0 0 0]);
            L.Velocities = [0 0 -1];
            sim.step(0.01, 40, -1);
            delete(sim);
            tc.verifyEqual(L.Positions, [0 0 0.99], "AbsTol", 1e-6);
            tc.verifyEqual(L.Velocities, [0 0 -1], "AbsTol", 1e-6);
        end
    end

    methods (Access = private)
        function requireLiquids(tc)
            % The engine on the path must know liquids (protocol 2, step U)
            tc.requireEngine;
            w = phxEngine('setup');
            cleanup = onCleanup(@() phxEngine('clear', w));
            g = phxEngine('add', w, 'particles', [0; 0; 0], 0.01, 0.01, false);
            try
                phxEngine('get', w, g, 'fluid');
            catch err
                tc.assumeFail("The engine on the path has no liquids (" + err.identifier + ").");
            end
        end

        function sim = simulate(tc, objects, gravity)
            % A simulation at the contact tolerances of centimetre parts
            arguments
                tc
                objects
                gravity (1, 3) double = [0 0 -9.81]
            end
            sim = phx.Simulation(objects, "Gravity", gravity, ...
                "EngineSettings", phx.engine.Settings("LengthScale", 0.02));
            tc.addTeardown(@() tc.deleteIfValid(sim));
        end
    end

end
